#include <stdio.h>
#include "esp_err.h" // for esp_err_t

//freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//driver
#include "driver/gpio.h"
#include "driver/ledc.h"

//components
#include "mpu6050.h"
#include "motor.h"
#include "nvs_utils.h"
#include "wifi_ws.h"

/* simple PID controller used for roll/pitch/yaw rate control */
typedef struct {
    float kp, ki, kd;
    float prev_error;
    float integ;
} PIDController;

static float pid_update(PIDController *pid, float error, float dt)
{
    pid->integ += error * dt;
    float deriv = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;
}


/* store the most recently received control values; updated from the
   websocket callback and read by the main loop */
static CompactDronePacket current_cmd = {0};

/* callback invoked by wifi_ws when a packet arrives */
static void control_packet_received(CompactDronePacket pkt)
{
    /* simply update the global and print; a real implementation would
       convert these values to motor outputs or use them in the attitude
       controller */
    current_cmd = pkt;
    printf("[cb] thr=%u roll=%u pitch=%u yaw=%u\n",
           pkt.throttle, pkt.roll, pkt.pitch, pkt.yaw);
}

void app_main(void)
{
    printf("Funktioniert... vielleicht???\n");
    
    printf("Versuche i2c master init\n");
    i2c_master_init();
    printf("Versuche mpu6050 init\n");
    if (mpu6050_init() != ESP_OK) {
        printf("mpu6050_init failed\n");
    }

    printf("Versuche motor pwm init\n");
    if (motor_pwm_init() != ESP_OK) {
        printf("motor_pwm_init failed\n");
    }
    printf("Versuche motor channel init\n");
    if (motor_channel_init() != ESP_OK) {
        printf("motor_channel_init failed\n");
    }

    printf("Versuche wifi init\n");
    if (wifi_init_ap() != ESP_OK) {
        printf("wifi_init_ap failed\n");
    }
    printf("Versuche websocket server start\n");
    websocket_server_start();

    /* register the control packet handler so incoming binary frames are
       decoded into compactDronePacket values */
    register_control_callback(control_packet_received);

    int16_t accel[3], gyro[3];

    /* PID instances for rate control */
    PIDController roll_pid  = { .kp = 0.5f, .ki = 0.1f, .kd = 0.01f };
    PIDController pitch_pid = { .kp = 0.5f, .ki = 0.1f, .kd = 0.01f };
    PIDController yaw_pid   = { .kp = 0.8f, .ki = 0.1f, .kd = 0.02f };

    vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for sensor to stabilize

    /* run at 100Hz (10ms period) for PID */
    const float dt = 0.01f;
    while (1){
        /* decode the four 11‑bit command fields into convenient floating
           ranges.  roll/pitch/yaw map 0..2047 to -100..+100 (10.235 ≈ 2047/200)
           while throttle goes to a 0‑100% span for the motor driver. */
        float decoded_roll = ((float)current_cmd.roll / 10.235f) - 100.0f;
        float decoded_pitch = ((float)current_cmd.pitch / 10.235f) - 100.0f;
        float decoded_yaw = ((float)current_cmd.yaw / 10.235f) - 100.0f;
        float decoded_throttle = ((float)current_cmd.throttle / 20.47f); // 0..100

        /* read gyro rates immediately so PID uses newest data */
        mpu6050_read(accel, gyro);
        float measured_roll_rate = (float)gyro[0];
        float measured_pitch_rate = (float)gyro[1];
        float measured_yaw_rate = (float)gyro[2];

        /* compute errors and run PID controllers (rate loop) */
        float roll_error = decoded_roll - measured_roll_rate;
        float pitch_error = decoded_pitch - measured_pitch_rate;
        float yaw_error = decoded_yaw - measured_yaw_rate;

        float roll_term = pid_update(&roll_pid, roll_error, dt);
        float pitch_term = pid_update(&pitch_pid, pitch_error, dt);
        float yaw_term = pid_update(&yaw_pid, yaw_error, dt);

        printf("pid outputs R=%.2f P=%.2f Y=%.2f  errors R=%.2f P=%.2f Y=%.2f\n",
               roll_term, pitch_term, yaw_term,
               roll_error, pitch_error, yaw_error);

        /* mix the four channels using the PID outputs */
        float m1 = decoded_throttle + roll_term - pitch_term - yaw_term;
        float m2 = decoded_throttle - roll_term - pitch_term + yaw_term;
        float m3 = decoded_throttle + roll_term + pitch_term + yaw_term;
        float m4 = decoded_throttle - roll_term + pitch_term - yaw_term;

        /* helper to confine to [0,100] */
        static inline float clamp100(float x)
        {
            if (x < 0.0f) return 0.0f;
            if (x > 100.0f) return 100.0f;
            return x;
        }
        m1 = clamp100(m1);
        m2 = clamp100(m2);
        m3 = clamp100(m3);
        m4 = clamp100(m4);

        /* convert each channel to duty and send to driver */
        uint32_t d1 = (uint32_t)((m1 / 100.0f) * 256);
        uint32_t d2 = (uint32_t)((m2 / 100.0f) * 256);
        uint32_t d3 = (uint32_t)((m3 / 100.0f) * 256);
        uint32_t d4 = (uint32_t)((m4 / 100.0f) * 256);

        set_motor_power(LEDC_CHANNEL_0, d1);
        set_motor_power(LEDC_CHANNEL_1, d2);
        set_motor_power(LEDC_CHANNEL_2, d3);
        set_motor_power(LEDC_CHANNEL_3, d4);

        // print sensor and command information (gyro already updated above)
        printf("accel: X=%d Y=%d Z=%d  ", accel[0], accel[1], accel[2]);
        printf("gyro: X=%d Y=%d Z=%d\n", gyro[0], gyro[1], gyro[2]);
        printf("last cmd thr=%u roll=%u pitch=%u yaw=%u\n",
               current_cmd.throttle, current_cmd.roll,
               current_cmd.pitch, current_cmd.yaw);
        printf("decoded thr=%.1f roll=%.1f pitch=%.1f yaw=%.1f\n",
               decoded_throttle, decoded_roll, decoded_pitch, decoded_yaw);

        vTaskDelay((int)(dt * 1000) / portTICK_PERIOD_MS);
    }
}

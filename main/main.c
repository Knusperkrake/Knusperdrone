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

/* global PID instances for rate control - can be modified via websocket */
static PIDController roll_pid  = { .kp = 0.5f, .ki = 0.1f, .kd = 0.01f };
static PIDController pitch_pid = { .kp = 0.5f, .ki = 0.1f, .kd = 0.01f };
static PIDController yaw_pid   = { .kp = 0.8f, .ki = 0.1f, .kd = 0.02f };

/* global trim values for roll/pitch/yaw - can be modified via websocket */
static float roll_trim  = 0.0f;
static float pitch_trim = 0.0f;
static float yaw_trim   = 0.0f;


/* store the most recently received control values; updated from the
   websocket callback and read by the main loop */
static ControlPacket current_cmd = {0};

/* callback invoked by wifi_ws when a control packet arrives */
static void control_packet_received(ControlPacket pkt)
{
    current_cmd = pkt;
}

/* callback invoked by wifi_ws when a config packet arrives */
static void config_packet_received(uint8_t command, const uint8_t *data, size_t len, httpd_req_t *req)
{
    printf("[config] cmd=%u len=%zu\n", command, len);
    switch (command) {
        case CMD_READ_PID: // read all PID
            {
                // Send back 9 PID floats: roll P,I,D, pitch P,I,D, yaw P,I,D
                float pid_data[9] = {
                    roll_pid.kp, roll_pid.ki, roll_pid.kd,
                    pitch_pid.kp, pitch_pid.ki, pitch_pid.kd,
                    yaw_pid.kp, yaw_pid.ki, yaw_pid.kd
                };
                websocket_send_response((uint8_t*)pid_data, sizeof(pid_data));
                printf("Sent PID values\n");
            }
            break;
        case CMD_WRITE_PID: // write all PID
            {
                const float *pids = (const float*)(data + 1);
                // Set PID values: pids[0-2] for roll, [3-5] for pitch, [6-8] for yaw
                roll_pid.kp = pids[0];
                roll_pid.ki = pids[1];
                roll_pid.kd = pids[2];
                pitch_pid.kp = pids[3];
                pitch_pid.ki = pids[4];
                pitch_pid.kd = pids[5];
                yaw_pid.kp = pids[6];
                yaw_pid.ki = pids[7];
                yaw_pid.kd = pids[8];
                printf("Updated PID: roll P=%.3f I=%.3f D=%.3f\n", pids[0], pids[1], pids[2]);
            }
            break;
        case CMD_READ_BATTERY: // read battery
            // TODO: read actual battery voltage/current
            {
                float battery_voltage = 3.7f;  // placeholder
                websocket_send_response((uint8_t*)&battery_voltage, sizeof(battery_voltage));
                printf("Sent battery voltage: %.2fV\n", battery_voltage);
            }
            break;
        case CMD_READ_TRIM: // read trim
            {
                // Send back 3 trim floats: roll, pitch, yaw
                float trim_data[3] = { roll_trim, pitch_trim, yaw_trim };
                websocket_send_response((uint8_t*)trim_data, sizeof(trim_data));
                printf("Sent trim values: roll=%.3f pitch=%.3f yaw=%.3f\n", roll_trim, pitch_trim, yaw_trim);
            }
            break;
        case CMD_WRITE_TRIM: // write trim
            {
                const float *trims = (const float*)(data + 1);
                // Set trim values for roll, pitch, yaw
                roll_trim = trims[0];
                pitch_trim = trims[1];
                yaw_trim = trims[2];
                printf("Updated trim: roll=%.3f pitch=%.3f yaw=%.3f\n", trims[0], trims[1], trims[2]);
            }
            break;
        case CMD_TARE_GYRO: // tare gyro
            // TODO: implement gyro taring/calibration
            printf("Tare gyro request\n");
            break;
        default:
            printf("Unknown config command %u\n", command);
            break;
    }
}

static inline float clamp100(float x)
        {
            if (x < 0.0f) return 0.0f;
            if (x > 100.0f) return 100.0f;
            return x;
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

    /* Avoid static unused-variable warnings when the callbacks are only
       referenced via function pointer registration (Werror builds). */
    (void)roll_trim;
    (void)pitch_trim;
    (void)yaw_trim;

    /* register the control packet handler so incoming binary frames are
       decoded into ControlPacket values */
    register_control_callback(control_packet_received);
    register_config_callback(config_packet_received);

    int16_t accel[3], gyro[3];

    /* PID instances are now global and initialized above */

    vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for sensor to stabilize

    /* run at 100Hz (10ms period) for PID */
    const float dt = 0.01f;
    while (1){
        /* If no client is connected, zero out the commands to ensure motors stay off */
        if (!is_client_connected()) {
            current_cmd = (ControlPacket){0};
        }

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

        /* Suppress unused variable warnings for PID terms (not yet integrated into motor mix) */
        (void)roll_term;
        (void)pitch_term;
        (void)yaw_term;

        /*printf("pid outputs R=%.2f P=%.2f Y=%.2f  errors R=%.2f P=%.2f Y=%.2f\n",
               roll_term, pitch_term, yaw_term,
               roll_error, pitch_error, yaw_error);*/

        
        float m1 = decoded_throttle + decoded_roll - decoded_pitch - decoded_yaw;
        float m2 = decoded_throttle - decoded_roll - decoded_pitch + decoded_yaw;
        float m3 = decoded_throttle + decoded_roll + decoded_pitch + decoded_yaw;
        float m4 = decoded_throttle - decoded_roll + decoded_pitch - decoded_yaw;

        /* mix the four channels using the PID outputs + feedforward from decoded stick commands */
        /* unused for now, since the PID is not yet tuned and the drone would be unstable with it - just send stick commands directly to motors
        float m1 = decoded_throttle + decoded_roll + roll_term - decoded_pitch - pitch_term - decoded_yaw - yaw_term;
        float m2 = decoded_throttle - decoded_roll - roll_term - decoded_pitch - pitch_term + decoded_yaw + yaw_term;
        float m3 = decoded_throttle + decoded_roll + roll_term + decoded_pitch + pitch_term + decoded_yaw + yaw_term;
        float m4 = decoded_throttle - decoded_roll - roll_term + decoded_pitch + pitch_term - decoded_yaw - yaw_term;
        */
        
        /* helper to confine to [0,100] ()<this war migrated to earlier in the code
        static inline float clamp100(float x)
        {
            if (x < 0.0f) return 0.0f;
            if (x > 100.0f) return 100.0f;
            return x;
        }
        */
     
        
        m1 = clamp100(m1);
        m2 = clamp100(m2);
        m3 = clamp100(m3);
        m4 = clamp100(m4);

        /* convert each channel to duty and send to driver */
        uint32_t d1 = (uint32_t)((m1 / 100.0f) * 255);
        uint32_t d2 = (uint32_t)((m2 / 100.0f) * 255);
        uint32_t d3 = (uint32_t)((m3 / 100.0f) * 255);
        uint32_t d4 = (uint32_t)((m4 / 100.0f) * 255);

        // only set motor power if a client is connected. WILL STOP MIDAIR IF CLIENT DISCONNECTS
        if (is_client_connected)
        {
            set_motor_power(LEDC_CHANNEL_0, d1);
            set_motor_power(LEDC_CHANNEL_1, d2);
            set_motor_power(LEDC_CHANNEL_2, d3);
            set_motor_power(LEDC_CHANNEL_3, d4);
        } else {
            set_motor_power(LEDC_CHANNEL_0, 0);
            set_motor_power(LEDC_CHANNEL_1, 0);
            set_motor_power(LEDC_CHANNEL_2, 0);
            set_motor_power(LEDC_CHANNEL_3, 0);
        }

        //printf("duty: duty1=%ld duty2=%ld duty3=%ld duty4=%ld \n", d1, d2, d3, d4);

        // print sensor and command information (gyro already updated above)
        /*
        printf("accel: X=%d Y=%d Z=%d  ", accel[0], accel[1], accel[2]);
        printf("gyro: X=%d Y=%d Z=%d\n", gyro[0], gyro[1], gyro[2]);
        printf("last cmd thr=%u roll=%u pitch=%u yaw=%u\n",
               current_cmd.throttle, current_cmd.roll,
               current_cmd.pitch, current_cmd.yaw);
        printf("decoded thr=%.1f roll=%.1f pitch=%.1f yaw=%.1f\n",
               decoded_throttle, decoded_roll, decoded_pitch, decoded_yaw);*/

        vTaskDelay((int)(dt * 1000) / portTICK_PERIOD_MS);
    }
}
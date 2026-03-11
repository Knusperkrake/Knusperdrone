#include <stdio.h>

//freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//driver
#include "driver/gpio.h"
#include "driver/ledc.h"

//components
#include "mpu6050.h"
#include "motor.h"


void app_main(void)
{
    printf("Funktioniert... vielleicht???\n");
    
    printf("Versuche i2c master init\n");
    i2c_master_init();
    printf("Versuche mpu6050 init\n");
    mpu6050_init();

    printf("Versuche motor pwm init\n");
    motor_pwm_init();
    printf("Versuche motor channel init\n");
    motor_channel_init();

    int16_t accel[3], gyro[3];

    vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for sensor to stabilize

    while (1){
        set_motor_power(LEDC_CHANNEL_0, 256); // 100% duty cycle
        set_motor_power(LEDC_CHANNEL_1, 256); // 100% duty cycle
        set_motor_power(LEDC_CHANNEL_2, 256); // 100% duty cycle
        set_motor_power(LEDC_CHANNEL_3, 256); // 100% duty cycle

        // read sensor and print for debugging
        mpu6050_read(accel, gyro);
        printf("accel: X=%d Y=%d Z=%d  ", accel[0], accel[1], accel[2]);
        printf("gyro: X=%d Y=%d Z=%d\n", gyro[0], gyro[1], gyro[2]);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

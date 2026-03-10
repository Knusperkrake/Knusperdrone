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
    
    i2c_master_init();
    mpu6050_init();

    motor_pwm_init();
    motor_channel_init();

    int16_t accel[3], gyro[3];

    while (1){
        set_motor_power(LEDC_CHANNEL_0, 128); // 50% duty cycle
        set_motor_power(LEDC_CHANNEL_1, 128); // 50% duty cycle
        set_motor_power(LEDC_CHANNEL_2, 128); // 50% duty cycle
        set_motor_power(LEDC_CHANNEL_3, 128); // 50% duty cycle

        // read sensor and print for debugging
        mpu6050_read(accel, gyro);
        printf("accel: X=%d Y=%d Z=%d  ", accel[0], accel[1], accel[2]);
        printf("gyro: X=%d Y=%d Z=%d\n", gyro[0], gyro[1], gyro[2]);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

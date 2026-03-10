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

    while (1){
        set_motor_power(LEDC_CHANNEL_0, 128); // 50% duty cycle
        set_motor_power(LEDC_CHANNEL_1, 128); // 50% duty cycle
        set_motor_power(LEDC_CHANNEL_2, 128); // 50% duty cycle
        set_motor_power(LEDC_CHANNEL_3, 128); // 50% duty cycle

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

void app_main(void)
{
    printf("Funktioniert... vielleicht???\n");
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    while (1){
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#include "motor.h"

#include "driver/ledc.h"

#define motor1_pin 9 // front left
#define motor2_pin 8 // front right
#define motor3_pin 7 // back left
#define motor4_pin 44 // back right


void motor_pwm_init() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 10000, // 10 kHz
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

}

void motor_channel_init() {
    ledc_channel_config_t channel1 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor1_pin,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel1);

    ledc_channel_config_t channel2 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor2_pin,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel2);

    ledc_channel_config_t channel3 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor3_pin,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel3);

    ledc_channel_config_t channel4 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor4_pin,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel4);
}


void set_motor_power(int channel, uint8_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}
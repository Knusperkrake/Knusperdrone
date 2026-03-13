#include "motor.h"

#include "driver/ledc.h"
#include "esp_log.h"

#define motor1_pin 9 // front left
#define motor2_pin 8 // front right
#define motor3_pin 7 // back left
#define motor4_pin 44 // back right

static const char *TAG = "motor";

esp_err_t motor_pwm_init() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 10000, // 10 kHz
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LEDC timer configured");
    }
    return err;
}

esp_err_t motor_channel_init() {
    esp_err_t err;

    ledc_channel_config_t channel1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor1_pin,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&channel1);
    if (err == ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "channel1 pin %d not usable in high-speed mode, retrying low-speed", motor1_pin);
        channel1.speed_mode = LEDC_LOW_SPEED_MODE;
        err = ledc_channel_config(&channel1);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel1 config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor2_pin,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&channel2);
    if (err == ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "channel2 pin %d not usable in high-speed mode, retrying low-speed", motor2_pin);
        channel2.speed_mode = LEDC_LOW_SPEED_MODE;
        err = ledc_channel_config(&channel2);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel2 config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel3 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor3_pin,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&channel3);
    if (err == ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "channel3 pin %d not usable in high-speed mode, retrying low-speed", motor3_pin);
        channel3.speed_mode = LEDC_LOW_SPEED_MODE;
        err = ledc_channel_config(&channel3);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel3 config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel4 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor4_pin,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&channel4);
    if (err == ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "channel4 pin %d not usable in high-speed mode, retrying low-speed", motor4_pin);
        channel4.speed_mode = LEDC_LOW_SPEED_MODE;
        err = ledc_channel_config(&channel4);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel4 config failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "all motor channels configured");
    return ESP_OK;
}

void set_motor_power(int channel, uint8_t duty) {
    if (duty > 0xff) {
        ESP_LOGW(TAG, "duty value %u out of range", duty);
    }
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed for ch %d: %s", channel, esp_err_to_name(err));
    } else {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }
}
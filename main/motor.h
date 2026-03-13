#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "esp_err.h"  // for esp_err_t

/**
 * Initialize LEDC timer for PWM.  Returns ESP_OK or an error code; callers
 * should log or abort if it fails.
 */
esp_err_t motor_pwm_init(void);

/**
 * Configure the four motor channels.  Returns ESP_OK if all channels were
 * configured successfully; if any channel fails, the error is logged and
 * returned.
 */
esp_err_t motor_channel_init(void);

/**
 * Set motor duty cycle (0..255) on the given LEDC channel.
 */
void set_motor_power(int channel, uint8_t duty);

void i2c_master_init(void);

#endif
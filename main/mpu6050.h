#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "esp_err.h"

/**
 * Initialise the MPU6050 device.  Returns ESP_OK if the wake‑up write
 * succeeded or an error code otherwise.
 */
esp_err_t mpu6050_init(void);

void mpu6050_read(int16_t* accel, int16_t* gyro);

#endif
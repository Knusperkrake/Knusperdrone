#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

void mpu6050_init();
void mpu6050_read(int16_t* accel, int16_t* gyro);

#endif
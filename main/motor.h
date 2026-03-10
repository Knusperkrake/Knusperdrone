#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_pwm_init();
void motor_channel_init();
void set_motor_power(int channel, uint8_t duty);

#endif
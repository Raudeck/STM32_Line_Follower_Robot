#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

extern double kp;
extern double ki;
extern double kd;
extern int previous_value;
extern int integral;
extern int left_speed;
extern int right_speed;
extern int derivative;

void motor_control(size_t left_motor_speed, size_t right_motor_speed);
void PID_Handle();

#endif
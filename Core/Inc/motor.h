#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

extern double kp;
extern double ki;
extern double kd;
extern int previous_value;
extern double integral;
extern double left_speed;
extern double right_speed;
extern double derivative;

void motor_control(double left_motor_speed, double right_motor_speed);
void PID_Handle();

#endif
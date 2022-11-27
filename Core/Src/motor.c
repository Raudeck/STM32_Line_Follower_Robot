#include "motor.h"
#include <stdlib.h>

double kp = 50;
double ki = 0;
double kd = 0.0;
double left_speed = 10;
double right_speed = 10;
double integral = 0;
double derivative = 0;
int previous_value = 0;

enum SharpTurn{
    left,
    right
};

void left_motor_counterclockwise()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void left_motor_clockwise()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

void left_motor_stop()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void right_motor_counterclockwise()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void right_motor_clockwise()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void right_motor_stop()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void motor_control(double left_motor_speed, double right_motor_speed)
{
    const size_t ARR = 20000;
    double CCRx_left_value = left_motor_speed * ARR / 100;
    double CCRx_right_value = right_motor_speed * ARR / 100;
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CCRx_right_value);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CCRx_left_value);

    right_motor_counterclockwise();
    left_motor_counterclockwise();
}

void PID_Handle()
{
    size_t sensor_array[5];
    int error = 0;
    int init_left_motor_speed = 50;
    int init_right_motor_speed = 50;
    int proportional = 0;

    /* Read sensors */
    sensor_array[0] = HAL_GPIO_ReadPin(SENSOR_LEFTMOST_GPIO_Port, SENSOR_LEFTMOST_Pin);
    sensor_array[1] = HAL_GPIO_ReadPin(SENSOR_LEFT_GPIO_Port, SENSOR_LEFT_Pin);
    sensor_array[2] = HAL_GPIO_ReadPin(SENSOR_MID_GPIO_Port, SENSOR_MID_Pin);
    sensor_array[3] = HAL_GPIO_ReadPin(SENSOR_RIGHT_GPIO_Port, SENSOR_RIGHT_Pin);
    sensor_array[4] = HAL_GPIO_ReadPin(SENSOR_RIGHTMOST_GPIO_Port, SENSOR_RIGHTMOST_Pin);

    /* Calc errors */
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0
        && sensor_array[3] == 0 && sensor_array[4] == 1)
        error = 4;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0
        && sensor_array[3] == 1 && sensor_array[4] == 1)
        error = 3;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0
        && sensor_array[3] == 1 && sensor_array[4] == 0)
        error = 2;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 1
        && sensor_array[3] == 1 && sensor_array[4] == 0)
        error = 1;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 1
        && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = 0;
    if (sensor_array[0] == 0 && sensor_array[1] == 1 && sensor_array[2] == 1
        && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -1;
    if (sensor_array[0] == 0 && sensor_array[1] == 1 && sensor_array[2] == 0
        && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -2;
    if (sensor_array[0] == 1 && sensor_array[1] == 1 && sensor_array[2] == 0
        && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -3;
    if (sensor_array[0] == 1 && sensor_array[1] == 0 && sensor_array[2] == 0
        && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -4;
    
    /* PID */
    proportional = error;
    integral = integral + error;
    derivative = (error - previous_value);
    int output = kp * proportional + ki * integral + kd * derivative;
    previous_value = error;
    
    right_speed = init_right_motor_speed - output;
    left_speed = init_left_motor_speed + output;

    /* Speed */
    if (right_speed > 100)
        right_speed = 100;
    if (right_speed < 0)
        right_speed = 0;
    if (left_speed > 100)
        left_speed = 100;
    if (left_speed < 0)
        left_speed = 0;
    HAL_Delay(2);
}
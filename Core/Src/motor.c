#include "motor.h"
#include <stdlib.h>

double kp = 25;
// double ki = 0.003;
double ki = 0.001;
double kd = 100;
double integral = 0;
double derivative = 0;
int previous_value = 0;
size_t counter = 0;
size_t sharp_turn_counter = 0;

enum SharpTurn
{
    left,
    right
};

void right_motor_counterclockwise()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void right_motor_clockwise()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

void right_motor_stop()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void left_motor_counterclockwise()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void left_motor_clockwise()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void left_motor_stop()
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

void sharp_turn(enum SharpTurn position)
{
    switch (position)
    {
    case left:
        /* Reverse the car */
        right_motor_clockwise();
        left_motor_clockwise();
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 5000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 5000);
        HAL_Delay(50);
        /* Start turning left */
        motor_control(80, 0);
        HAL_Delay(850);
        motor_control(0, 0);
        break;
    case right:
        /* Reverse the car */
        right_motor_clockwise();
        left_motor_clockwise();
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 5000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 5000);
        HAL_Delay(50);
        /* Start turning right */
        motor_control(0, 80);
        HAL_Delay(850);
        /* Make it stop */
        motor_control(0, 0);
        break;
    }
}

void go_straight()
{
    motor_control(80,80);
    HAL_Delay(100);
}

void PID_Handle()
{
    size_t sensor_array[5];
    int error = 0;
    double init_left_motor_speed = 100;
    double init_right_motor_speed = 100;
    int proportional = 0;
    enum Status
    {
        UsePID,
        HandleSharpTurn,
        GoStraight,
        ReadAgain
    };

    enum Status status = UsePID;
    enum SharpTurn position = right;

    /* Read sensors */
    sensor_array[0] = HAL_GPIO_ReadPin(SENSOR_LEFTMOST_GPIO_Port, SENSOR_LEFTMOST_Pin);
    sensor_array[1] = HAL_GPIO_ReadPin(SENSOR_LEFT_GPIO_Port, SENSOR_LEFT_Pin);
    sensor_array[2] = HAL_GPIO_ReadPin(SENSOR_MID_GPIO_Port, SENSOR_MID_Pin);
    sensor_array[3] = HAL_GPIO_ReadPin(SENSOR_RIGHT_GPIO_Port, SENSOR_RIGHT_Pin);
    sensor_array[4] = HAL_GPIO_ReadPin(SENSOR_RIGHTMOST_GPIO_Port, SENSOR_RIGHTMOST_Pin);

    /* Calc errors */
    if (sensor_array[0] == 1 && sensor_array[1] == 1 && sensor_array[2] == 1 && sensor_array[3] == 1 && sensor_array[4] == 1)
        status = GoStraight;
    if (sensor_array[0] == 1 && sensor_array[1] == 1 && sensor_array[2] == 1 && sensor_array[3] == 0 && sensor_array[4] == 0)
    {
        status = ReadAgain;
        position = right;
    }
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 1 && sensor_array[3] == 1 && sensor_array[4] == 1)
    {
        status = ReadAgain;
        position = left;
    }

    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0 && sensor_array[3] == 0 && sensor_array[4] == 1)
        error = 4;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0 && sensor_array[3] == 1 && sensor_array[4] == 1)
        error = 3;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0 && sensor_array[3] == 1 && sensor_array[4] == 0)
        error = 2;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 1 && sensor_array[3] == 1 && sensor_array[4] == 0)
        error = 1;
    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 1 && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = 0;
    if (sensor_array[0] == 0 && sensor_array[1] == 1 && sensor_array[2] == 1 && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -1;
    if (sensor_array[0] == 0 && sensor_array[1] == 1 && sensor_array[2] == 0 && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -2;
    if (sensor_array[0] == 1 && sensor_array[1] == 1 && sensor_array[2] == 0 && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -3;
    if (sensor_array[0] == 1 && sensor_array[1] == 0 && sensor_array[2] == 0 && sensor_array[3] == 0 && sensor_array[4] == 0)
        error = -4;

    if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0 && sensor_array[3] == 0 && sensor_array[4] == 0)
    {
        status = ReadAgain;
        counter++;
    }

    if (counter == 16381)
    {
        counter = 0;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 750);
        HAL_Delay(500);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1999);
    }

    if (sharp_turn_counter == 132)
    {
        sharp_turn_counter = 0;
        status = HandleSharpTurn;
    }

    size_t i = 0;

    for (i = 0; i <= 1; i++)
    {
        switch (status)
        {
        case UsePID:
            /* PID */
            proportional = error;
            integral = integral + error;
            derivative = (error - previous_value);
            double output = kp * proportional + ki * integral + kd * derivative;
            previous_value = error;
            double right_speed = init_right_motor_speed - output;
            double left_speed = init_left_motor_speed + output;

            /* Speed */
            if (right_speed > 100)
                right_speed = 100;
            if (right_speed < 0)
                right_speed = 0;
            if (left_speed > 100)
                left_speed = 100;
            if (left_speed < 0)
                left_speed = 0;
            motor_control(left_speed, right_speed);
            i++;
            break;
        case GoStraight:
            go_straight();
            i++;
            break;
        case HandleSharpTurn:
            sharp_turn(position);
            i++;
            break;
        case ReadAgain:
            sensor_array[0] = HAL_GPIO_ReadPin(SENSOR_LEFTMOST_GPIO_Port, SENSOR_LEFTMOST_Pin);
            sensor_array[1] = HAL_GPIO_ReadPin(SENSOR_LEFT_GPIO_Port, SENSOR_LEFT_Pin);
            sensor_array[2] = HAL_GPIO_ReadPin(SENSOR_MID_GPIO_Port, SENSOR_MID_Pin);
            sensor_array[3] = HAL_GPIO_ReadPin(SENSOR_RIGHT_GPIO_Port, SENSOR_RIGHT_Pin);
            sensor_array[4] = HAL_GPIO_ReadPin(SENSOR_RIGHTMOST_GPIO_Port, SENSOR_RIGHTMOST_Pin);
            if (sensor_array[0] == 1 && sensor_array[1] == 1 && sensor_array[2] == 1 && sensor_array[3] == 0 && sensor_array[4] == 0)
            {
                sharp_turn_counter++;
                position = right;
            }
            else if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 1 && sensor_array[3] == 1 && sensor_array[4] == 1)
            {
                sharp_turn_counter++;
                position = left;
            }
            else if (sensor_array[0] == 0 && sensor_array[1] == 0 && sensor_array[2] == 0 && sensor_array[3] == 0 && sensor_array[4] == 0)
            {
                status = UsePID;
                counter++;
                switch (error)
                {
                    case -4:
                        error = -5;
                    break;
                    case 4:
                        error = 5;
                    break;
                }
            }
            else
            {
                i++;
                counter = 0;
                sharp_turn_counter = 0;
            }
            break;
        }
    }
}
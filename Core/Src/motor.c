#include "motor.h"

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

void motor_control(size_t left_motor_speed, size_t right_motor_speed)
{
    const size_t ARR = 20000;
    size_t CCRx_left_value = (size_t)(left_motor_speed * ARR / 100);
    size_t CCRx_right_value = (size_t)(left_motor_speed * ARR / 100);
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CCRx_right_value);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CCRx_left_value);

    right_motor_counterclockwise();
    left_motor_counterclockwise();
}
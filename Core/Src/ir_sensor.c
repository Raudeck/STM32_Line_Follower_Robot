#include "sensor.h"

void Get_IR_Value(ADC_HandleTypeDef *hadc1, uint32_t *buffer)
{
    HAL_ADC_Start_DMA(hadc1, buffer, 1);
    return ;
}
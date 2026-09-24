/**
 * @file    timebase.c
 * @brief   1 ms application timebase, driven by the TIM7 update interrupt.
 */
#include "timebase.h"
#include "tim.h"

static volatile uint32_t time_ms = 0;

void Timebase_Start(void)
{
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
        Error_Handler();
    }
}

uint32_t Timebase_GetMs(void)
{
    return time_ms;
}

// TIM7: 96 MHz / (1 + 1) / (47999 + 1) = 1 kHz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        time_ms++;
    }
}

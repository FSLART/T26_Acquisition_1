/**
 * @file    debug.c
 * @brief   Debug output over USART1: printf retarget.
 */
#include "debug.h"
#include "usart.h"

int _write(int file, char *data, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, HAL_MAX_DELAY);
    return len;
}

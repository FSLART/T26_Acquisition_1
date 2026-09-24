/**
 * @file    debug.h
 * @brief   Debug output over USART1 (printf is retargeted in debug.c).
 *          UART TX is blocking: ~87 us per character at 115200 baud, keep prints short and rare.
 */
#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

#define DEBUG_PRINT_ENABLE  1   // 0 -> all DEBUG_PRINTF calls compile to nothing

#if DEBUG_PRINT_ENABLE
#define DEBUG_PRINTF(...)   printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)   ((void)0)
#endif

#endif /* DEBUG_H */

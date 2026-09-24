/**
 * @file    timebase.h
 * @brief   1 ms application timebase, driven by the TIM7 update interrupt.
 */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

void Timebase_Start(void);
uint32_t Timebase_GetMs(void);

#endif /* TIMEBASE_H */

/**
 * @file    wheel_speed.h
 * @brief   Wheel speed sensors, tooth period measured with TIM3 input capture.
 */
#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>

typedef enum {
    WHEEL_1 = 0,    // PC8 - TIM3_CH3
    WHEEL_2,        // PC9 - TIM3_CH4
    WHEEL_COUNT
} WheelId;

void WheelSpeed_Init(void);
float WheelSpeed_GetRpm(WheelId wheel);

#endif /* WHEEL_SPEED_H */

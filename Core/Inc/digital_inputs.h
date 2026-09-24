/**
 * @file    digital_inputs.h
 * @brief   Digital inputs sent in AQT1: RES (PC8) and BOTS (PC7), no pull (external circuit).
 */
#ifndef DIGITAL_INPUTS_H
#define DIGITAL_INPUTS_H

#include <stdint.h>

void DigitalInputs_Update(void);    // Call before packing AQT1
uint8_t DigitalInputs_GetRes(void);     // 1 = pin high
uint8_t DigitalInputs_GetBots(void);    // 1 = pin high

#endif /* DIGITAL_INPUTS_H */

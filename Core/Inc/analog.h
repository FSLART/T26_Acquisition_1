/**
 * @file    analog.h
 * @brief   Analog inputs: APPS1, APPS2, front brake pressure, MCU temperature, VDDA.
 *          ADC1 scans continuously into a DMA buffer, Analog_Update() (10 ms) averages the last
 *          ADC_OVERSAMPLE samples of each channel.
 */
#ifndef ANALOG_H
#define ANALOG_H

#include <stdint.h>

void Analog_Init(void);
void Analog_Update(void);               // Call every 10 ms

float Analog_GetApps1Counts(void);      // Averaged ADC counts, 0..4095 (PA7)
float Analog_GetApps2Counts(void);      // Averaged ADC counts, 0..4095 (PB0)
float Analog_GetBrakeCounts(void);      // Averaged ADC counts, 0..4095 (PB1)
float Analog_GetBrakePressureBar(void); // Front brake pressure [bar], 0..140
float Analog_GetVdda(void);             // Measured analog supply [V]
int32_t Analog_GetMcuTempC(void);       // MCU die temperature [C]

#endif /* ANALOG_H */

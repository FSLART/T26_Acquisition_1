/**
 * @file    wheel_speed.c
 * @brief   Wheel speed sensors, tooth period measured with TIM3 input capture.
 */
#include "wheel_speed.h"
#include "tim.h"
#include "timebase.h"

#define WHEEL_TIMER_FREQ_HZ     1000000.0f  // TIM3: 96 MHz / (95 + 1) = 1 MHz -> 1 us resolution
#define WHEEL_NUMBER_OF_TEETH   20.0f       // Disc teeth count
#define WHEEL_STANDSTILL_MS     300         // No tooth for this long -> 0 RPM

// 16-bit timer at 1 MHz wraps every 65.5 ms, a longer tooth gap can't be measured
// (it would alias into a wrong, too high RPM). ~46 RPM is the slowest measurable speed.
#define WHEEL_MAX_TOOTH_GAP_MS  60

typedef struct {
    uint32_t channel;                       // TIM3 channel
    HAL_TIM_ActiveChannel active_channel;   // Matching htim->Channel value in the callback
    volatile uint16_t last_capture;         // Timer reading from previous tooth
    volatile float rpm;                     // Last calculated speed
    volatile uint8_t first_capture;         // Ignore first pulse at startup
    volatile uint32_t last_pulse_ms;        // Timestamp of the last detected tooth
} WheelSensor;

static WheelSensor wheels[WHEEL_COUNT] = {
    [WHEEL_1] = { .channel = TIM_CHANNEL_3, .active_channel = HAL_TIM_ACTIVE_CHANNEL_3, .first_capture = 1 },
    [WHEEL_2] = { .channel = TIM_CHANNEL_4, .active_channel = HAL_TIM_ACTIVE_CHANNEL_4, .first_capture = 1 },
};

void WheelSpeed_Init(void)
{
    for (uint32_t i = 0; i < WHEEL_COUNT; i++) {
        if (HAL_TIM_IC_Start_IT(&htim3, wheels[i].channel) != HAL_OK) {
            Error_Handler();
        }
    }
}

float WheelSpeed_GetRpm(WheelId wheel)
{
    if (wheel >= WHEEL_COUNT) {
        return 0.0f;
    }

    // Standstill Detection: 0 RPM if no pulse received recently
    if (Timebase_GetMs() - wheels[wheel].last_pulse_ms > WHEEL_STANDSTILL_MS) {
        return 0.0f;
    }

    return wheels[wheel].rpm;
}

static void WheelSpeed_OnCapture(WheelSensor *wheel, uint16_t capture)
{
    uint32_t now = Timebase_GetMs();

    if (wheel->first_capture || (now - wheel->last_pulse_ms) > WHEEL_MAX_TOOTH_GAP_MS) {
        // No valid previous tooth, or too slow to measure
        wheel->first_capture = 0;
        wheel->rpm = 0.0f;
    } else {
        // 16-bit subtraction handles counter overflow (65535 -> 0) automatically
        uint16_t delta_us = (uint16_t)(capture - wheel->last_capture);

        if (delta_us > 0) {
            // RPM = (60 s * 1,000,000 us) / (teeth * delta_us)
            wheel->rpm = (60.0f * WHEEL_TIMER_FREQ_HZ) / (WHEEL_NUMBER_OF_TEETH * (float)delta_us);
        }
    }

    wheel->last_capture = capture;
    wheel->last_pulse_ms = now;
}

// Hardware Capture Interrupt: Triggered automatically on rising edges (PC8 & PC9)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) {
        return;
    }

    for (uint32_t i = 0; i < WHEEL_COUNT; i++) {
        if (htim->Channel == wheels[i].active_channel) {
            WheelSpeed_OnCapture(&wheels[i], (uint16_t)HAL_TIM_ReadCapturedValue(htim, wheels[i].channel));
        }
    }
}

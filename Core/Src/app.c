/**
 * @file    app.c
 * @brief   Application entry: module init and the cooperative task scheduler.
 */
#include "app.h"
#include "main.h"
#include "watchdog.h"
#include "timebase.h"
#include "wheel_speed.h"
#include "analog.h"
#include "can_bus.h"
#include "can_messages.h"
#include "debug.h"

static void App_Task10ms(void);
static void App_Task100ms(void);
static void App_Task500ms(void);

void App_Init(void)
{
    Watchdog_Init();
    DEBUG_PRINTF("\r\nT26 Acquisition 1 boot, reset cause: %s, SYSCLK: %lu Hz\r\n",
                 Watchdog_GetResetCauseName(), HAL_RCC_GetSysClockFreq());

    Timebase_Start();
    WheelSpeed_Init();
    Analog_Init();
    CanBus_Init();
}

void App_Loop(void)
{
    static uint32_t last_10ms = 0;
    static uint32_t last_100ms = 0;
    static uint32_t last_500ms = 0;

    uint32_t now = Timebase_GetMs();

    // Immediate tasks
    CanBus_Service();

    if (now - last_10ms >= 10) {
        last_10ms = now;
        App_Task10ms();
    }

    if (now - last_100ms >= 100) {
        last_100ms = now;
        App_Task100ms();
    }

    if (now - last_500ms >= 500) {
        last_500ms = now;
        App_Task500ms();
    }
}

static void App_Task10ms(void)
{
    // Refreshed only while the scheduler runs: a stuck loop or a dead 1 ms timebase resets the board
    Watchdog_Refresh();

    Analog_Update();

    CanMsg_SendAppsAdcRaw();    // CAN2 - powertrain
    CanMsg_SendAqt1();          // CAN1 - autonomous
}

static void App_Task100ms(void)
{
    HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin); // HEARTBEAT

    CanBus_CheckHealth();
}

// Debug output, kept slow: blocking UART prints delay the 10 ms task
static void App_Task500ms(void)
{
    DEBUG_PRINTF("Wheel 1: %.1f RPM | Wheel 2: %.1f RPM\r\n",
                 WheelSpeed_GetRpm(WHEEL_1), WheelSpeed_GetRpm(WHEEL_2));
    DEBUG_PRINTF("APPS1: %.1f | APPS2: %.1f | Brake: %.1f bar | VDDA: %.3f V | MCU: %ld C\r\n",
                 Analog_GetApps1Counts(), Analog_GetApps2Counts(), Analog_GetBrakePressureBar(),
                 Analog_GetVdda(), Analog_GetMcuTempC());
    CanBus_PrintStatus();
}

/**
 * @file    app.c
 * @brief   Application entry: module init and the cooperative task scheduler.
 *          10 ms sensors + CAN TX, 100 ms heartbeat + CAN health, 500 ms debug print.
 *          (The original firmware sent at 50 ms, too coarse for the APPS.)
 */
#include "app.h"
#include "main.h"
#include "watchdog.h"
#include "timebase.h"
#include "analog.h"
#include "digital_inputs.h"
#include "can_bus.h"
#include "can_messages.h"
#include "debug.h"
#include <string.h>

// Debug snapshot of the whole board: add "acq1" to STM32CubeIDE Live Expressions
typedef struct {
	uint32_t time_ms;               // Uptime [ms]
	const char *reset_cause;        // Why the board last reset

	struct {
		float apps1_counts;         // Filtered ADC counts, PA7
		float apps2_counts;         // Filtered ADC counts, PB0
		float brake_counts;         // Filtered ADC counts, PB1
		float brake_bar;            // Front brake pressure
		float vdda_v;               // Measured analog supply
		int32_t mcu_temp_c;         // MCU die temperature
	} analog;

	struct {
		uint8_t res;                // PC8
		uint8_t bots;               // PC7
	} io;

	struct {
		uint8_t tx_050[8];          // Last payload sent on CAN2 (APPS_ADC_Raw 0x50)
		uint8_t tx_710[8];          // Last payload sent on CAN1 (AQT1 0x710)
		CanBusStatus can1;
		CanBusStatus can2;
	} can;
} Acq1_Debug;

Acq1_Debug acq1;

static void App_Task10ms(void);
static void App_Task100ms(void);
static void App_Task500ms(void);
static void App_UpdateDebug(void);

void App_Init(void) {
	Watchdog_Init();
	acq1.reset_cause = Watchdog_GetResetCauseName();
	DEBUG_PRINTF("\r\nT26 Acquisition 1 boot, reset cause: %s, SYSCLK: %lu Hz\r\n", acq1.reset_cause,
			HAL_RCC_GetSysClockFreq());

	Timebase_Start();
	Analog_Init();
	CanBus_Init();
}

void App_Loop(void) {
	static uint32_t last_10ms = 0;
	static uint32_t last_100ms = 0;
	static uint32_t last_500ms = 0;

	uint32_t now = Timebase_GetMs();

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

static void App_Task10ms(void) {
	// Refreshed only while the scheduler runs: a stuck loop or a dead 1 ms timebase resets the board
	Watchdog_Refresh();

	Analog_Update();
	DigitalInputs_Update();

	CanMsg_SendAppsAdcRaw();    // CAN2 - powertrain
	CanMsg_SendAqt1();          // CAN1 - autonomous

	App_UpdateDebug();
}

static void App_Task100ms(void) {
	HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin); // HEARTBEAT

	CanBus_CheckHealth();       // Prints on UART only when a bus state changes
}

// Debug output, kept slow: blocking UART prints delay the other tasks
static void App_Task500ms(void) {
	DEBUG_PRINTF("RES = %d | BOTS = %d\r\n", DigitalInputs_GetRes(), DigitalInputs_GetBots());
	DEBUG_PRINTF("Brake: %.2f bar (adc %.1f) | APPS1: %.1f | APPS2: %.1f | VDDA: %.3f V | MCU: %ld C\r\n",
			Analog_GetBrakePressureBar(), Analog_GetBrakeCounts(), Analog_GetApps1Counts(), Analog_GetApps2Counts(),
			Analog_GetVdda(), Analog_GetMcuTempC());
}

// Copies everything into acq1 for Live Expressions (called every 10 ms)
static void App_UpdateDebug(void) {
	acq1.time_ms = Timebase_GetMs();

	acq1.analog.apps1_counts = Analog_GetApps1Counts();
	acq1.analog.apps2_counts = Analog_GetApps2Counts();
	acq1.analog.brake_counts = Analog_GetBrakeCounts();
	acq1.analog.brake_bar = Analog_GetBrakePressureBar();
	acq1.analog.vdda_v = Analog_GetVdda();
	acq1.analog.mcu_temp_c = Analog_GetMcuTempC();

	acq1.io.res = DigitalInputs_GetRes();
	acq1.io.bots = DigitalInputs_GetBots();

	memcpy(acq1.can.tx_050, CanMsg_GetAppsAdcRawPayload(), sizeof(acq1.can.tx_050));
	memcpy(acq1.can.tx_710, CanMsg_GetAqt1Payload(), sizeof(acq1.can.tx_710));
	acq1.can.can1 = *CanBus_GetStatus(CAN_BUS_AUTONOMOUS);
	acq1.can.can2 = *CanBus_GetStatus(CAN_BUS_POWERTRAIN);
}

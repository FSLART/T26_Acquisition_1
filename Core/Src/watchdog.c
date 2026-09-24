/**
 * @file    watchdog.c
 * @brief   Independent watchdog (IWDG) refresh and reset cause reporting.
 *          IWDG itself is configured by CubeMX (MX_IWDG_Init), timeout = 64 * 1000 / 32 kHz LSI ~ 2 s.
 */
#include "watchdog.h"
#include "iwdg.h"

static ResetCause reset_cause = RESET_CAUSE_UNKNOWN;

// RCC CSR reset flags are sticky until cleared. PINRST is set on every reset (NRST is driven low
// internally) and PORRST sets BORRST too, so the most specific flag is checked first.
static ResetCause Watchdog_ReadResetCause(void) {
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		return RESET_CAUSE_IWDG;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
		return RESET_CAUSE_WWDG;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
		return RESET_CAUSE_LOW_POWER;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
		return RESET_CAUSE_SOFTWARE;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		return RESET_CAUSE_POWER_ON;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
		return RESET_CAUSE_BROWN_OUT;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		return RESET_CAUSE_PIN;
	}
	return RESET_CAUSE_UNKNOWN;
}

void Watchdog_Init(void) {
	// IWDG keeps counting while the core is halted by the debugger (breakpoint, pause, step)
	// and resets the board. Freeze it during debug halt, no effect when no debugger is attached.
	__HAL_DBGMCU_FREEZE_IWDG();

	reset_cause = Watchdog_ReadResetCause();
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

void Watchdog_Refresh(void) {
	HAL_IWDG_Refresh(&hiwdg);
}

ResetCause Watchdog_GetResetCause(void) {
	return reset_cause;
}

const char* Watchdog_GetResetCauseName(void) {
	switch (reset_cause) {
	case RESET_CAUSE_IWDG:
		return "IWDG (firmware hung)";
	case RESET_CAUSE_WWDG:
		return "WWDG";
	case RESET_CAUSE_LOW_POWER:
		return "low-power";
	case RESET_CAUSE_SOFTWARE:
		return "software / debugger";
	case RESET_CAUSE_POWER_ON:
		return "power-on";
	case RESET_CAUSE_BROWN_OUT:
		return "brown-out";
	case RESET_CAUSE_PIN:
		return "NRST pin";
	default:
		return "unknown";
	}
}

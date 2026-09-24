/**
 * @file    watchdog.h
 * @brief   Independent watchdog (IWDG) refresh and reset cause reporting.
 *          IWDG itself is configured by CubeMX (MX_IWDG_Init), timeout = 64 * 1000 / 32 kHz LSI ~ 2 s.
 */
#ifndef WATCHDOG_H
#define WATCHDOG_H

typedef enum {
	RESET_CAUSE_UNKNOWN = 0,
	RESET_CAUSE_IWDG,           // Independent watchdog: firmware hung (Error_Handler, HardFault, blocking loop)
	RESET_CAUSE_WWDG,           // Window watchdog
	RESET_CAUSE_LOW_POWER,      // Low-power management
	RESET_CAUSE_SOFTWARE,       // NVIC_SystemReset() or debugger reset
	RESET_CAUSE_POWER_ON,       // Power-on
	RESET_CAUSE_BROWN_OUT,      // Supply dropped below BOR threshold
	RESET_CAUSE_PIN             // NRST pin
} ResetCause;

void Watchdog_Init(void);       // Call first in App_Init()
void Watchdog_Refresh(void);
ResetCause Watchdog_GetResetCause(void);
const char* Watchdog_GetResetCauseName(void);

#endif /* WATCHDOG_H */

/**
 * @file    can_messages.c
 * @brief   Messages sent by Acquisition_Board1, packed with the cantools DBC code.
 *          CAN2 powertrain: APPS_ADC_Raw (0x50)
 *          CAN1 autonomous: AQT1 (0x710)
 */
#include "can_messages.h"
#include "can_bus.h"
#include "analog.h"
#include "../DBC/autonomous_t26.h"
#include "../DBC/powertrain_t26.h"

// APPS_ADC_Raw (0x50) on CAN2 - powertrain: averaged ADC counts, pedal scaling done by the VCU
void CanMsg_SendAppsAdcRaw(void) {
	struct powertrain_t26_apps_adc_raw_t msg = { 0 };
	uint8_t data[8];

	msg.apps1_raw = powertrain_t26_apps_adc_raw_apps1_raw_encode(Analog_GetApps1Counts());
	msg.apps2_raw = powertrain_t26_apps_adc_raw_apps2_raw_encode(Analog_GetApps2Counts());

	int len = powertrain_t26_apps_adc_raw_pack(data, &msg, sizeof(data));
	if (len > 0) {
		CanBus_Send(CAN_BUS_POWERTRAIN, POWERTRAIN_T26_APPS_ADC_RAW_FRAME_ID, data, (uint8_t) len);
	}
}

// AQT1 (0x710) on CAN1 - autonomous: front brake pressure
void CanMsg_SendAqt1(void) {
	struct autonomous_t26_aqt1_t msg = { 0 };
	uint8_t data[8];

	msg.frt_brk_press = autonomous_t26_aqt1_frt_brk_press_encode(Analog_GetBrakePressureBar());
	msg.res = 0;    // Not wired to this board
	msg.bots = 0;   // Not wired to this board

	int len = autonomous_t26_aqt1_pack(data, &msg, sizeof(data));
	if (len > 0) {
		CanBus_Send(CAN_BUS_AUTONOMOUS, AUTONOMOUS_T26_AQT1_FRAME_ID, data, (uint8_t) len);
	}
}

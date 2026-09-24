/**
 * @file    can_messages.c
 * @brief   Messages sent by Acquisition_Board1, packed with the cantools DBC code.
 *          CAN2 powertrain: APPS_ADC_Raw (0x50)
 *          CAN1 autonomous: AQT1 (0x710)
 */
#include "can_messages.h"
#include "can_bus.h"
#include "analog.h"
#include "digital_inputs.h"
#include "../DBC/autonomous_t26.h"
#include "../DBC/powertrain_t26.h"

// Last payload of each message, kept for the acq1 debug struct
static uint8_t apps_adc_raw_data[8];
static uint8_t aqt1_data[8];

// APPS_ADC_Raw (0x50) on CAN2 - powertrain: filtered ADC counts x10, pedal scaling done by the VCU
void CanMsg_SendAppsAdcRaw(void) {
	struct powertrain_t26_apps_adc_raw_t msg = { 0 };

	msg.apps1_raw = powertrain_t26_apps_adc_raw_apps1_raw_encode(Analog_GetApps1Counts());
	msg.apps2_raw = powertrain_t26_apps_adc_raw_apps2_raw_encode(Analog_GetApps2Counts());

	int len = powertrain_t26_apps_adc_raw_pack(apps_adc_raw_data, &msg, sizeof(apps_adc_raw_data));
	if (len > 0) {
		CanBus_Send(CAN_BUS_POWERTRAIN, POWERTRAIN_T26_APPS_ADC_RAW_FRAME_ID, apps_adc_raw_data, (uint8_t) len);
	}
}

// AQT1 (0x710) on CAN1 - autonomous: front brake pressure x10, RES (PC8) bit 16, BOTS (PC7) bit 17
void CanMsg_SendAqt1(void) {
	struct autonomous_t26_aqt1_t msg = { 0 };

	msg.frt_brk_press = autonomous_t26_aqt1_frt_brk_press_encode(Analog_GetBrakePressureBar());
	msg.res = DigitalInputs_GetRes();
	msg.bots = DigitalInputs_GetBots();

	int len = autonomous_t26_aqt1_pack(aqt1_data, &msg, sizeof(aqt1_data));
	if (len > 0) {
		CanBus_Send(CAN_BUS_AUTONOMOUS, AUTONOMOUS_T26_AQT1_FRAME_ID, aqt1_data, (uint8_t) len);
	}
}

const uint8_t* CanMsg_GetAppsAdcRawPayload(void) {
	return apps_adc_raw_data;
}

const uint8_t* CanMsg_GetAqt1Payload(void) {
	return aqt1_data;
}

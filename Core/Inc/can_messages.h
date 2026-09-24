/**
 * @file    can_messages.h
 * @brief   Messages sent by Acquisition_Board1, packed with the cantools DBC code.
 *          CAN2 powertrain: APPS_ADC_Raw (0x50)
 *          CAN1 autonomous: AQT1 (0x710)
 */
#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include <stdint.h>

void CanMsg_SendAppsAdcRaw(void);
void CanMsg_SendAqt1(void);

const uint8_t* CanMsg_GetAppsAdcRawPayload(void);   // 8 bytes, last payload sent
const uint8_t* CanMsg_GetAqt1Payload(void);         // 8 bytes, last payload sent

#endif /* CAN_MESSAGES_H */

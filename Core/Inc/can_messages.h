/**
 * @file    can_messages.h
 * @brief   Messages sent by Acquisition_Board1, packed with the cantools DBC code.
 *          CAN2 powertrain: APPS_ADC_Raw (0x50)
 *          CAN1 autonomous: AQT1 (0x710)
 */
#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

void CanMsg_SendAppsAdcRaw(void);
void CanMsg_SendAqt1(void);

#endif /* CAN_MESSAGES_H */

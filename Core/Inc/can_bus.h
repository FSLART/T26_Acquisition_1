/**
 * @file    can_bus.h
 * @brief   CAN1/CAN2 driver layer: RX filters, latest-value TX, error monitoring and recovery.
 *          Only standard-ID data frames are accepted, extended-ID and remote frames are
 *          rejected by the hardware filter.
 */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>

typedef enum {
	CAN_BUS_AUTONOMOUS = 0,     // CAN1
	CAN_BUS_POWERTRAIN,         // CAN2
	CAN_BUS_COUNT
} CanBusId;

typedef enum {
	CAN_STATE_ERROR_ACTIVE = 0, // Normal
	CAN_STATE_ERROR_WARNING,    // TEC or REC >= 96
	CAN_STATE_ERROR_PASSIVE,    // TEC or REC > 127
	CAN_STATE_BUS_OFF           // TEC > 255, recovers automatically (AutoBusOff)
} CanBusState;

// Bus health, one per bus (also copied into the acq1 debug struct for Live Expressions)
typedef struct {
	CanBusState state;
	uint8_t tec;                // Transmit error counter
	uint8_t rec;                // Receive error counter
	uint8_t last_error;         // Last error code (ESR.LEC) seen since the previous check, 0 = none
	uint8_t fault;              // 1 while the bus reports a problem
	uint32_t tx_queued;         // Frames handed to a TX mailbox
	uint32_t tx_dropped;        // Frames not sent: controller not running or all 3 mailboxes busy
	uint32_t tx_aborted;        // Older copies of a frame aborted to send the latest value
	uint32_t restarts;          // Controller restarts done by CanBus_CheckHealth()
} CanBusStatus;

void CanBus_Init(void);
void CanBus_Send(CanBusId bus, uint32_t std_id, const uint8_t *data, uint8_t len);
void CanBus_CheckHealth(void);  // Call every 100 ms: error counters, UART report on change, recovery
const CanBusStatus* CanBus_GetStatus(CanBusId bus);

#endif /* CAN_BUS_H */

/**
 * @file    can_bus.h
 * @brief   CAN1/CAN2 driver layer: RX filters, TX queues, error monitoring and recovery.
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
    CAN_STATE_BUS_OFF           // TEC > 255, recovers automatically (ABOM)
} CanBusState;

typedef struct {
    CanBusState state;
    uint8_t tec;                // Transmit error counter
    uint8_t rec;                // Receive error counter
    uint8_t last_error;         // Last error code (ESR.LEC) seen since previous health check, 0 = none
    uint32_t tx_dropped;        // Frames dropped because the TX queue was full
    uint32_t tx_aborted;        // Times stuck mailboxes were aborted (no ACK / bus-off)
    uint32_t restarts;          // Times the peripheral was re-initialised
} CanBusStatus;

void CanBus_Init(void);
void CanBus_Send(CanBusId bus, uint32_t std_id, const uint8_t *data, uint8_t len);
void CanBus_Service(void);      // Call every main loop pass: moves queued frames into free mailboxes
void CanBus_CheckHealth(void);  // Call every 100 ms: error counters + recovery
const CanBusStatus *CanBus_GetStatus(CanBusId bus);
void CanBus_PrintStatus(void);

#endif /* CAN_BUS_H */

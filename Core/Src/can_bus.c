/**
 * @file    can_bus.c
 * @brief   CAN1/CAN2 driver layer: RX filters, TX queues, error monitoring and recovery.
 *          Only standard-ID data frames are accepted, extended-ID and remote frames are
 *          rejected by the hardware filter.
 */
#include "can_bus.h"
#include "can.h"
#include "debug.h"
#include "timebase.h"
#include <string.h>

#define CAN_TX_QUEUE_LEN            16      // Frames per bus
#define CAN_TX_STUCK_TIMEOUT_MS     20      // All mailboxes busy this long -> abort them (no ACK / bus-off)
#define CAN_RESTART_INTERVAL_MS     1000    // Min time between restarts, a failing start blocks ~10 ms

// CAN1 and CAN2 share 28 filter banks: 0..13 -> CAN1, 14..27 -> CAN2 (also the reset value).
// Must be the same in every filter config, HAL rewrites the split on every call.
#define CAN_SLAVE_START_FILTER_BANK 14

typedef struct {
	uint32_t std_id;
	uint8_t len;
	uint8_t data[8];
} CanTxFrame;

typedef struct {
	CAN_HandleTypeDef *hcan;
	const char *name;
	uint32_t filter_bank;

	// TX ring buffer, only used from the main loop (no ISR access, no locking needed)
	CanTxFrame queue[CAN_TX_QUEUE_LEN];
	uint8_t head;               // Oldest frame
	uint8_t count;

	uint8_t tx_stuck;           // All mailboxes busy
	uint32_t tx_stuck_since_ms;

	uint32_t last_restart_ms;

	CanBusStatus status;
} CanBus;

static CanBus buses[CAN_BUS_COUNT] =
		{ [CAN_BUS_AUTONOMOUS] = { .hcan = &hcan1, .name = "CAN1 autonomous", .filter_bank = 0 }, [CAN_BUS_POWERTRAIN] = { .hcan = &hcan2, .name = "CAN2 powertrain", .filter_bank = CAN_SLAVE_START_FILTER_BANK }, };

// Accept every standard-ID data frame, reject all extended-ID and remote frames.
// 32-bit mask mode: only the IDE and RTR bits are compared (both must be 0), ID bits are don't-care.
static HAL_StatusTypeDef CanBus_ConfigFilter(CanBus *bus) {
	CAN_FilterTypeDef filter = { 0 };

	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterBank = bus->filter_bank;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;                            // IDE = 0 (standard), RTR = 0 (data)
	filter.FilterMaskIdHigh = 0x0000;                       // Any ID
	filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;   // Must match: IDE and RTR bits
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = CAN_SLAVE_START_FILTER_BANK;

	return HAL_CAN_ConfigFilter(bus->hcan, &filter);
}

static HAL_StatusTypeDef CanBus_Start(CanBus *bus) {
	if (CanBus_ConfigFilter(bus) != HAL_OK) {
		return HAL_ERROR;
	}
	return HAL_CAN_Start(bus->hcan);
}

// Peripheral re-init, used when HAL lost the "started" state (start timeout, init error...).
// No HAL_CAN_DeInit() on purpose: the generated CAN1 MspDeInit switches off the CAN1 clock
// without updating the shared counter, which would kill CAN2 and never re-enable CAN1.
static void CanBus_Restart(CanBus *bus) {
	HAL_CAN_Stop(bus->hcan);

	if (HAL_CAN_Init(bus->hcan) == HAL_OK) {
		CanBus_Start(bus);
	}

	bus->status.restarts++;
}

void CanBus_Init(void) {
	// A failed start (e.g. bus held dominant) is not fatal: CanBus_CheckHealth() keeps retrying
	for (uint32_t i = 0; i < CAN_BUS_COUNT; i++) {
		CanBus_Start(&buses[i]);
	}
}

void CanBus_Send(CanBusId id, uint32_t std_id, const uint8_t *data, uint8_t len) {
	if (id >= CAN_BUS_COUNT || len > 8) {
		return;
	}

	CanBus *bus = &buses[id];

	// Queue full: drop the oldest frame, fresh data is worth more
	if (bus->count == CAN_TX_QUEUE_LEN) {
		bus->head = (bus->head + 1) % CAN_TX_QUEUE_LEN;
		bus->count--;
		bus->status.tx_dropped++;
	}

	CanTxFrame *frame = &bus->queue[(bus->head + bus->count) % CAN_TX_QUEUE_LEN];
	frame->std_id = std_id;
	frame->len = len;
	memcpy(frame->data, data, len);
	bus->count++;
}

// All 3 mailboxes pending for too long means nobody ACKs or the node is bus-off.
// Abort them so stale frames don't go out first once the bus is back.
static void CanBus_AbortStuckMailboxes(CanBus *bus, uint32_t now) {
	if (HAL_CAN_GetState(bus->hcan) != HAL_CAN_STATE_LISTENING || HAL_CAN_GetTxMailboxesFreeLevel(bus->hcan) > 0) {
		bus->tx_stuck = 0;
		return;
	}

	if (!bus->tx_stuck) {
		bus->tx_stuck = 1;
		bus->tx_stuck_since_ms = now;
		return;
	}

	if (now - bus->tx_stuck_since_ms >= CAN_TX_STUCK_TIMEOUT_MS) {
		HAL_CAN_AbortTxRequest(bus->hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
		bus->status.tx_aborted++;
		bus->tx_stuck = 0;
	}
}

void CanBus_Service(void) {
	uint32_t now = Timebase_GetMs();

	for (uint32_t i = 0; i < CAN_BUS_COUNT; i++) {
		CanBus *bus = &buses[i];

		CanBus_AbortStuckMailboxes(bus, now);

		while (bus->count > 0 && HAL_CAN_GetTxMailboxesFreeLevel(bus->hcan) > 0) {
			CanTxFrame *frame = &bus->queue[bus->head];
			CAN_TxHeaderTypeDef header = { 0 };
			uint32_t mailbox;

			header.StdId = frame->std_id;
			header.IDE = CAN_ID_STD;
			header.RTR = CAN_RTR_DATA;
			header.DLC = frame->len;
			header.TransmitGlobalTime = DISABLE;

			if (HAL_CAN_AddTxMessage(bus->hcan, &header, frame->data, &mailbox) != HAL_OK) {
				break;  // Peripheral not running, keep the frame for later
			}

			bus->head = (bus->head + 1) % CAN_TX_QUEUE_LEN;
			bus->count--;
		}
	}
}

// Error state straight from the ESR register (HAL only tracks it with error IRQs enabled)
static void CanBus_ReadErrorStatus(CanBus *bus) {
	CAN_TypeDef *can = bus->hcan->Instance;
	uint32_t esr = can->ESR;
	uint8_t lec = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;

	bus->status.tec = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
	bus->status.rec = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;

	// LEC = 7 is written by us below, so 7 means no new error since the previous check
	bus->status.last_error = (lec == 7) ? 0 : lec;
	can->ESR = CAN_ESR_LEC;     // Only LEC is writable in ESR

	if (esr & CAN_ESR_BOFF) {
		bus->status.state = CAN_STATE_BUS_OFF;
	} else if (esr & CAN_ESR_EPVF) {
		bus->status.state = CAN_STATE_ERROR_PASSIVE;
	} else if (esr & CAN_ESR_EWGF) {
		bus->status.state = CAN_STATE_ERROR_WARNING;
	} else {
		bus->status.state = CAN_STATE_ERROR_ACTIVE;
	}
}

void CanBus_CheckHealth(void) {
	uint32_t now = Timebase_GetMs();

	for (uint32_t i = 0; i < CAN_BUS_COUNT; i++) {
		CanBus *bus = &buses[i];

		CanBus_ReadErrorStatus(bus);

		// Bus-off recovers by itself (AutoBusOff), only a peripheral that is not running needs a restart
		if (HAL_CAN_GetState(bus->hcan) != HAL_CAN_STATE_LISTENING && now - bus->last_restart_ms >= CAN_RESTART_INTERVAL_MS) {
			bus->last_restart_ms = now;
			CanBus_Restart(bus);
		}

		HAL_CAN_ResetError(bus->hcan);
	}
}

const CanBusStatus* CanBus_GetStatus(CanBusId id) {
	if (id >= CAN_BUS_COUNT) {
		return NULL;
	}
	return &buses[id].status;
}

static const char* CanBus_StateName(CanBusState state) {
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "OK";
	case CAN_STATE_ERROR_WARNING:
		return "WARNING";
	case CAN_STATE_ERROR_PASSIVE:
		return "PASSIVE";
	case CAN_STATE_BUS_OFF:
		return "BUS-OFF";
	default:
		return "?";
	}
}

static const char* CanBus_ErrorName(uint8_t lec) {
	static const char *const names[] = { "none", "stuff", "form", "ack", "bit recessive", "bit dominant", "crc", "none" };
	return (lec < 8) ? names[lec] : "?";
}

void CanBus_PrintStatus(void) {
	for (uint32_t i = 0; i < CAN_BUS_COUNT; i++) {
		const CanBus *bus = &buses[i];
		const CanBusStatus *s = &bus->status;

		DEBUG_PRINTF("%s: %s TEC=%u REC=%u err=%s dropped=%lu aborted=%lu restarts=%lu\r\n", bus->name, CanBus_StateName(s->state), s->tec, s->rec, CanBus_ErrorName(s->last_error), s->tx_dropped, s->tx_aborted, s->restarts);
	}
}

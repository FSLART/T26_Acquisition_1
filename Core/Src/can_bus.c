/**
 * @file    can_bus.c
 * @brief   CAN1/CAN2 driver layer: RX filters, latest-value TX, error monitoring and recovery.
 *          Only standard-ID data frames are accepted, extended-ID and remote frames are
 *          rejected by the hardware filter.
 */
#include "can_bus.h"
#include "can.h"
#include "debug.h"
#include "timebase.h"
#include <string.h>

#define CAN_RESTART_INTERVAL_MS     1000    // Min time between restarts, a failing start blocks ~10-20 ms

// CAN1 and CAN2 share 28 filter banks: 0..13 -> CAN1, 14..27 -> CAN2 (also the reset value).
// Must be the same in every filter config, HAL rewrites the split on every call.
#define CAN_SLAVE_START_FILTER_BANK 14

typedef struct {
	CAN_HandleTypeDef *hcan;
	const char *name;
	uint32_t filter_bank;
	uint32_t last_restart_ms;
	CanBusState reported_state;     // Last state/error printed on UART, to report only changes
	uint8_t reported_error;
	CanBusStatus status;
} CanBus;

static CanBus buses[CAN_BUS_COUNT] = {
	[CAN_BUS_AUTONOMOUS] = { .hcan = &hcan1, .name = "CAN1 autonomous", .filter_bank = 0 },
	[CAN_BUS_POWERTRAIN] = { .hcan = &hcan2, .name = "CAN2 powertrain", .filter_bank = CAN_SLAVE_START_FILTER_BANK },
};

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

// Peripheral re-init in place, used when HAL lost the "started" state (start timeout, init error...).
// No HAL_CAN_DeInit() on purpose: the generated CAN1 MspDeInit switches off the CAN1 clock
// without updating the shared counter, which would kill CAN2 and never re-enable CAN1.
static void CanBus_Restart(CanBus *bus) {
	if (HAL_CAN_GetState(bus->hcan) == HAL_CAN_STATE_LISTENING) {
		HAL_CAN_Stop(bus->hcan);
	}

	if (HAL_CAN_Init(bus->hcan) == HAL_OK) {
		CanBus_Start(bus);
	}

	bus->status.restarts++;
}

void CanBus_Init(void) {
	// A failed start (e.g. transceiver unpowered, bus held dominant) is not fatal:
	// CanBus_CheckHealth() keeps retrying
	for (uint32_t i = 0; i < CAN_BUS_COUNT; i++) {
		CanBus_Start(&buses[i]);
	}
}

// Sends only the latest value: any older copy of the same frame still waiting in a mailbox
// is aborted first, so nothing is buffered. Never blocks, never calls Error_Handler.
void CanBus_Send(CanBusId id, uint32_t std_id, const uint8_t *data, uint8_t len) {
	static const uint32_t tme[3] = { CAN_TSR_TME0, CAN_TSR_TME1, CAN_TSR_TME2 };
	static const uint32_t mailboxes[3] = { CAN_TX_MAILBOX0, CAN_TX_MAILBOX1, CAN_TX_MAILBOX2 };

	if (id >= CAN_BUS_COUNT || len > 8) {
		return;
	}

	CanBus *bus = &buses[id];
	CAN_TypeDef *can = bus->hcan->Instance;
	CAN_TxHeaderTypeDef header = { 0 };
	uint32_t mailbox;

	if (HAL_CAN_GetState(bus->hcan) != HAL_CAN_STATE_LISTENING) {
		bus->status.tx_dropped++;
		return;
	}

	// Abort pending mailboxes that hold an older value of this frame
	uint32_t tsr = can->TSR;
	for (uint32_t i = 0; i < 3; i++) {
		uint32_t tir = can->sTxMailBox[i].TIR;
		uint32_t pending_id = (tir & CAN_TI0R_STID) >> CAN_TI0R_STID_Pos;

		if (((tsr & tme[i]) == 0U) && ((tir & CAN_TI0R_IDE) == 0U) && (pending_id == std_id)) {
			HAL_CAN_AbortTxRequest(bus->hcan, mailboxes[i]);
			bus->status.tx_aborted++;
		}
	}

	header.StdId = std_id;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = len;
	header.TransmitGlobalTime = DISABLE;

	// A frame already on the wire cannot be aborted; it finishes and the new value uses another
	// mailbox. Only if all 3 are busy is this value dropped (next period sends a fresh one).
	if (HAL_CAN_AddTxMessage(bus->hcan, &header, data, &mailbox) != HAL_OK) {
		bus->status.tx_dropped++;
		return;
	}

	bus->status.tx_queued++;
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

// Error state straight from the ESR register (error interrupts are not enabled)
static void CanBus_ReadErrorStatus(CanBus *bus) {
	CAN_TypeDef *can = bus->hcan->Instance;
	uint32_t esr = can->ESR;
	uint8_t lec = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;

	bus->status.tec = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
	bus->status.rec = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
	bus->status.last_error = (lec == 7) ? 0 : lec;  // 7 = set by software, not a bus error

	// Clear the last error code so the next check only sees new errors
	CLEAR_BIT(can->ESR, CAN_ESR_LEC);

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

// Report only when the state or the error changes, so the UART is not flooded
static void CanBus_ReportChange(CanBus *bus) {
	CanBusStatus *s = &bus->status;

	if (s->state == bus->reported_state && s->last_error == bus->reported_error) {
		return;
	}

	DEBUG_PRINTF("%s: %s TEC=%u REC=%u err=%s\r\n", bus->name, CanBus_StateName(s->state), s->tec, s->rec,
			CanBus_ErrorName(s->last_error));

	bus->reported_state = s->state;
	bus->reported_error = s->last_error;
}

void CanBus_CheckHealth(void) {
	uint32_t now = Timebase_GetMs();

	for (uint32_t i = 0; i < CAN_BUS_COUNT; i++) {
		CanBus *bus = &buses[i];
		uint8_t running = (HAL_CAN_GetState(bus->hcan) == HAL_CAN_STATE_LISTENING);

		CanBus_ReadErrorStatus(bus);
		bus->status.fault = !running || bus->status.state != CAN_STATE_ERROR_ACTIVE || bus->status.last_error != 0;
		CanBus_ReportChange(bus);

		// Bus-off recovers by itself (AutoBusOff), only a controller that is not running needs a restart
		if (!running && now - bus->last_restart_ms >= CAN_RESTART_INTERVAL_MS) {
			bus->last_restart_ms = now;
			DEBUG_PRINTF("%s: not running, restarting\r\n", bus->name);
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

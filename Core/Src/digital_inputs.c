/**
 * @file    digital_inputs.c
 * @brief   Digital inputs sent in AQT1: RES (PC8) and BOTS (PC7), no pull (external circuit).
 */
#include "digital_inputs.h"
#include "main.h"

static uint8_t res = 0;
static uint8_t bots = 0;

void DigitalInputs_Update(void) {
	res = (HAL_GPIO_ReadPin(RES_GPIO_Port, RES_Pin) == GPIO_PIN_SET) ? 1 : 0;
	bots = (HAL_GPIO_ReadPin(BOTS_GPIO_Port, BOTS_Pin) == GPIO_PIN_SET) ? 1 : 0;
}

uint8_t DigitalInputs_GetRes(void) {
	return res;
}

uint8_t DigitalInputs_GetBots(void) {
	return bots;
}

/*
 * keyboard.c
 *
 *  Created on: Nov 17, 2024
 *      Author: andryssssss
 */

#include "keyboard.h"

HAL_StatusTypeDef KBRD_Init(void) {
	HAL_StatusTypeDef status = PCA9538_SetDefaultConfig(KEYBOARD_I2C_ADDR);
	if (status != HAL_OK) {
		// UART_Transmit("Can't set keyboard default config\n");
		return status;
	}
	uint8_t out_def = 0x00;
	status = PCA9538_Write_Register(KEYBOARD_I2C_ADDR, OUTPUT_PORT, &out_def);
	if (status != HAL_OK) {
		// UART_Transmit("Can't set keyboard output port\n");
	}
	return status;
}

bool KBRD_IsSet(uint16_t* state, uint16_t bit) {
	return (*state & bit) > 0;
}

void KBRD_Set(uint16_t* state, uint16_t bit) {
	*state = (*state | bit);
}

void KBRD_Reset(uint16_t* state, uint16_t bit) {
	*state = (*state & (~bit));
}

void KBRD_ResetAll(uint16_t* state) {
	*state = 0;
}

HAL_StatusTypeDef KBRD_GetState(uint16_t* state) {
	static uint32_t pull_delay = 3; // delay between write/read register signals

	static uint32_t last_time = 0;
	static uint32_t minimum_delay = 20;

	static uint16_t prev_state = 0;

	KBRD_ResetAll(state);

	uint16_t cur_state;
	KBRD_ResetAll(&cur_state);

	while (HAL_GetTick() < last_time + minimum_delay) { }
	last_time = HAL_GetTick();

	HAL_StatusTypeDef status;
	uint8_t input_port;

	for (uint8_t i = 0; i < 4; i++) { // row
		uint8_t conf = ~(0x01 << i);

		status = PCA9538_Write_Register(KEYBOARD_I2C_ADDR, OUTPUT_PORT, &conf);
		if (status != HAL_OK) {
			// UART_Transmit("Can't write keyboard config register\n");
			return status;
		}

		HAL_Delay(pull_delay);

		status = PCA9538_Read_Inputs(KEYBOARD_I2C_ADDR, &input_port);
		if (status != HAL_OK) {
			// UART_Transmit("Can't read keyboard input port\n");
			return status;
		}

		input_port = (input_port >> 4) & 0x7; // column
		if (input_port != 0) {
			if ((input_port & 0x4) == 0) {
				KBRD_Set(&cur_state, BTN_9 << i);
			}
			if ((input_port & 0x2) == 0) {
				KBRD_Set(&cur_state, BTN_5 << i);
			}
			if ((input_port & 0x1) == 0) {
				KBRD_Set(&cur_state, BTN_1 << i);
			}
		}

		HAL_Delay(pull_delay);
	}

	*state = cur_state & (~prev_state);
	prev_state = cur_state;

	return HAL_OK;
}

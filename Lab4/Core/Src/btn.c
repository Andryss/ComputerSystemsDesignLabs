/*
 * btn.c
 *
 *  Created on: Sep 29, 2024
 *      Author: andryssssss
 */

#include "btn.h"

bool BTN_IsSet(uint16_t* state, uint16_t bit) {
	return (*state & bit) > 0;
}

void BTN_Set(uint16_t* state, uint16_t bit) {
	*state = (*state | bit);
}

void BTN_Reset(uint16_t* state, uint16_t bit) {
	*state = (*state & (~bit));
}

void BTN_ResetAll(uint16_t* state) {
	*state = 0;
}

void BTN_GetState(uint16_t* state) {
	static uint32_t up_threshold = 3000;
	static uint32_t down_threshold = 200;

	static uint32_t last_time = 0;
	static uint32_t minimum_delay = 20;

	static uint32_t reset_time = 1;
	static uint32_t set_time = 0;

	BTN_ResetAll(state);

	while (HAL_GetTick() < last_time + minimum_delay) { }
	last_time = HAL_GetTick();

	int current_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);

	if (current_state == 0) {
		BTN_Set(state, BTN_DOWN);

		if (reset_time != 0) {
			BTN_Set(state, BTN_DOWN_EVENT);

			reset_time = 0;
			set_time = HAL_GetTick();
		}
	} else {
		BTN_Set(state, BTN_UP);

		if (set_time == 0) {
			uint32_t up_time = HAL_GetTick() - reset_time;
			if (up_time > up_threshold) {
				BTN_Set(state, BTN_EOF);
			}
		} else {
			BTN_Set(state, BTN_UP_EVENT);

			uint32_t down_time = HAL_GetTick() - set_time;
			if (down_time < down_threshold) {
				BTN_Set(state, BTN_SHORT_PRESS);
			} else {
				BTN_Set(state, BTN_LONG_PRESS);
			}

			set_time = 0;
			reset_time = HAL_GetTick();
		}
	}
}

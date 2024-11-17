/*
 * keyboard.h
 *
 *  Created on: Nov 17, 2024
 *      Author: andryssssss
 */

#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

#include <stdbool.h>
#include "pca9538.h"

#define KEYBOARD_I2C_ADDR 0xE2

/*
 * 1  5  9
 * 2  6  10
 * 3  7  11
 * 4  8  12
 */

// First column
#define BTN_1 0x001
#define BTN_2 0x002
#define BTN_3 0x004
#define BTN_4 0x008
// Second column
#define BTN_5 0x010
#define BTN_6 0x020
#define BTN_7 0x040
#define BTN_8 0x080
// Third column
#define BTN_9 0x100
#define BTN_10 0x200
#define BTN_11 0x400
#define BTN_12 0x800

HAL_StatusTypeDef KBRD_Init(void);
HAL_StatusTypeDef KBRD_GetState(uint16_t*);
bool KBRD_IsSet(uint16_t*, uint16_t);

#endif /* INC_KEYBOARD_H_ */

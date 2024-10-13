/*
 * btn.h
 *
 *  Created on: Sep 29, 2024
 *      Author: andryssssss
 */

#ifndef INC_BTN_H_
#define INC_BTN_H_

#include "gpio.h"
#include <stdint.h>


#define BTN_DOWN 0x1
#define BTN_UP 0x2
#define BTN_DOWN_EVENT 0x4
#define BTN_UP_EVENT 0x8
#define BTN_SHORT_PRESS 0x10
#define BTN_LONG_PRESS 0x20
#define BTN_EOF 0x40


_Bool BTN_IsSet(uint16_t*, uint16_t);
void BTN_GetState(uint16_t*);

#endif /* INC_BTN_H_ */

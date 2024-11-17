/*
 * buf.h
 *
 *  Created on: Oct 13, 2024
 *      Author: andryssssss
 */

#ifndef INC_BUF_H_
#define INC_BUF_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "cmsis_gcc.h"

struct Buffer {
	uint8_t data[2048];
	uint16_t begin;
	uint16_t end;
	bool isEmpty;
};

void buf_init(struct Buffer*);
void buf_push(struct Buffer*, uint8_t*, size_t);
bool buf_pop(struct Buffer*, uint8_t*, size_t);
void buf_push_itsafe(struct Buffer*, uint8_t*, size_t);
bool buf_pop_itsafe(struct Buffer*, uint8_t*, size_t);

#endif /* INC_BUF_H_ */

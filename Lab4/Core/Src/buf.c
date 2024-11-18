/*
 * buf.c
 *
 *  Created on: Oct 13, 2024
 *      Author: andryssssss
 */
#include "buf.h"

void buf_init(struct Buffer* buf) {
	buf->data[0] = '\0';
	buf->begin = 0;
	buf->end = 0;
	buf->isEmpty = true;
}

bool is_buf_empty(struct Buffer* buf) {
	return buf->isEmpty;
}

bool is_buf_full(struct Buffer* buf) {
	return ((buf->end + 1 == buf->begin) || (buf->begin == 0 && buf->end == 2048 - 1));
}

void buf_push(struct Buffer* buf, uint8_t* data, size_t size) {
	size_t written = 0;
	while (written < size) {
		if (is_buf_full(buf)) {
			break;
		}
		buf->end = (buf->end + 1) % sizeof(buf->data);
		buf->data[buf->end] = *data;
		data++;
		written++;
	}
	if (!is_buf_full(buf)) {
		buf->end = (buf->end + 1) % sizeof(buf->data);
	}
	buf->data[buf->end] = '\0';
	buf->isEmpty = false;
}

bool buf_pop(struct Buffer* buf, uint8_t* data, size_t size) {
	if (is_buf_empty(buf)) {
		return false;
	}
	buf->begin = (buf->begin + 1) % sizeof(buf->data);
	size_t read = 0;
	while (buf->data[buf->begin] != '\0') {
		if (read < size) {
			data[read] = buf->data[buf->begin];
			read++;
		}
		buf->begin = (buf->begin + 1) % sizeof(buf->data);
	}
	data[read + (read == size ? -1 : 0)] = '\0';
	buf->isEmpty = (buf->begin == buf->end);
	return true;
}

void buf_push_itsafe(struct Buffer* buf, uint8_t* data, size_t size) {
	uint32_t pmask = __get_PRIMASK();
	__disable_irq();
	buf_push(buf, data, size);
	__set_PRIMASK(pmask);
}

bool buf_pop_itsafe(struct Buffer* buf, uint8_t* data, size_t size) {
	uint32_t pmask = __get_PRIMASK();
	__disable_irq();
	bool result = buf_pop(buf, data, size);
	__set_PRIMASK(pmask);
	return result;
}
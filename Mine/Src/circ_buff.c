/*
 * circ_buff.c
 *
 *  Created on: 8 sty 2023
 *      Author: domis
 */


#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stm32f4xx_hal.h>

#include "circ_buff.h"


static inline bool f_cb_isFull(t_cb_circBuffer *cb)
{
	return (cb->head == ((cb->tail + 1) % cb->length));
}

static inline bool f_cb_isEmpty(t_cb_circBuffer *cb)
{
	return (cb->head == cb->tail);
}


//=============== PUBLIC FUNCTIONS ==================

uint16_t f_cb_checkSize(t_cb_circBuffer *cb)
{
	uint16_t size;

	__disable_irq();
	if(cb->tail >= cb->head) size = cb->tail - cb->head + 1;
	else size = cb->length - cb->head - cb->tail;
	__enable_irq();

	return size;
}

bool f_cb_enqueue(t_cb_circBuffer *cb, const void *pData, uint16_t size)
{
	if (f_cb_isFull(cb)) return false;

	__disable_irq();
	memcpy(cb->pbuffAddress + cb->tail * size, pData, size);
	cb->tail = (cb->tail + 1) % cb->length;
	__enable_irq();

	return true;
}

void *f_cb_dequeue(t_cb_circBuffer *cb, void *pData, uint16_t size)
{
	if(f_cb_isEmpty(cb)) return NULL;

	__disable_irq();
	memcpy(pData, cb->pbuffAddress + cb->head * size, size);
	cb->head = (cb->head + 1) % cb->length;
	__enable_irq();

	return pData;
}

bool f_cb_tryEnqueue(t_cb_circBuffer *cb, const void *pData, uint16_t size)
{
	bool isOk;
	uint8_t tries = 0;

	for(tries = 0; tries < ENQUEUE_TRIES; tries++)
	{
		isOk = f_cb_enqueue(cb, pData, size);
		if(isOk) break;
		HAL_Delay(5);
	}

	if(tries == ENQUEUE_TRIES) isOk = false;

	return isOk;
}





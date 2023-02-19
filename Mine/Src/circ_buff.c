/*
 * circ_buff.c
 *
 *  Created on: 8 sty 2023
 *      Author: domis
 */


#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>

#include "circ_buff.h"


static inline bool f_cb_is_full(t_circ_buffer *cb)
{
	return (cb->head == ((cb->tail + 1) % MAX_BUFFER_SIZE));
}

static inline bool f_cb_is_empty(t_circ_buffer *cb)
{
	return (cb->head == cb->tail);
}


uint8_t f_cb_check_size(t_circ_buffer *cb)
{
	uint8_t size;

	__disable_irq();
	if(cb->tail >= cb->head) size = cb->tail - cb->head + 1;
	else size = MAX_BUFFER_SIZE - cb->head - cb->tail;
	__enable_irq();

	return size;
}

bool f_cb_enqueue(t_circ_buffer *cb, void *element)
{
	__disable_irq();
	if (f_cb_is_full(cb)) return false;
	cb->elements[cb->tail] = element;
	cb->tail = (cb->tail+1) % MAX_BUFFER_SIZE;
	__enable_irq();

	return true;
}

void *f_cb_dequeue(t_circ_buffer *cb)
{
	void *element;

	__disable_irq();
	if(f_cb_is_empty(cb)) return NULL;
	element = cb->elements[cb->head];
	cb->head = (cb->head+1) % MAX_BUFFER_SIZE;
	__enable_irq();

	return element;
}

bool f_cb_try_enqueue(t_circ_buffer *cb, void *element)
{
	bool isOk;
	uint8_t tries = 0;

	for(tries = 0; tries < ENQUEUE_TRIES; tries++)
	{
		isOk = f_cb_enqueue(cb, element);
		if(isOk) break;
		HAL_Delay(5);
	}

	if(tries == ENQUEUE_TRIES) isOk = false;

	return isOk;
}





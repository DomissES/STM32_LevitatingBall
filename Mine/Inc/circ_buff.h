/*
 * circ_buff.h
 *
 *  Created on: 8 sty 2023
 *      Author: domis
 */

#ifndef INC_CIRC_BUFF_H_
#define INC_CIRC_BUFF_H_

#include <stdbool.h>
#include <stdint.h>

#define ENQUEUE_TRIES 3
#define BUFFER_SIZE 16

typedef volatile struct
{
	void *buffAddress;
	uint16_t length;
	uint16_t head;
	uint16_t tail;

}t_cb_circBuffer;

// buffer should be two dimensional: [buffer size][data size of one element]

uint16_t f_cb_checkSize(t_cb_circBuffer *cb);
bool f_cb_enqueue(t_cb_circBuffer *cb, const void *element, uint16_t size);
void *f_cb_dequeue(t_cb_circBuffer *cb, void *element,  uint16_t size);
bool f_cb_tryEnqueue(t_cb_circBuffer *cb, const void *element, uint16_t size);

#endif /* INC_CIRC_BUFF_H_ */

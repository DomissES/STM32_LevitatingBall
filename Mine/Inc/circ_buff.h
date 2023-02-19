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
#define MAX_BUFFER_SIZE 16

typedef volatile struct
{
	uint8_t head;
	uint8_t tail;
	void *elements[MAX_BUFFER_SIZE];
}t_circ_buffer;

uint8_t f_cb_check_size(t_circ_buffer *cb);
bool f_cb_enqueue(t_circ_buffer *cb, void *element);
void *f_cb_dequeue(t_circ_buffer *cb);
bool f_cb_try_enqueue(t_circ_buffer *cb, void *element);

#endif /* INC_CIRC_BUFF_H_ */

/*
 * pid.h
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct
{
	uint16_t mili_P;
	uint16_t mili_I;
	uint16_t mili_D;
	uint16_t div_min;
	uint16_t div_max;
}t_pid_Parameter;

uint16_t f_pid_CalculateThrottle(uint16_t set, uint16_t get, t_pid_Parameter param);


#endif /* INC_PID_H_ */

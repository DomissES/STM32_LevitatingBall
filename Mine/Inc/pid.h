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
	uint16_t Kp;
	uint16_t Ki;
	uint16_t Kd;
	int32_t I_minRange;
	int32_t I_maxRange;
	int32_t minRange;
	int32_t maxRange;
}t_pid_Parameter;

typedef struct
{
	int32_t integral;
	int32_t lastError;
	int32_t lastInput;
	int32_t output;
	int32_t pValue;
	int32_t iValue;
	int32_t dValue;
}t_pid_Control;

t_pid_Control* f_pid_CalculateThrottle(int32_t setPoint, int32_t input, t_pid_Control* Ctrl, t_pid_Parameter* Param);


#endif /* INC_PID_H_ */

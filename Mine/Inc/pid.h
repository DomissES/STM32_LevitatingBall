/*
 * pid.h
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// NOTE:
// float variables are approx 1% slower than fix variables, (261 cycles vs 240 cycles with two function calls)

#include "main.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float I_minRange;
	float I_maxRange;
	float minRange;
	float maxRange;
}t_pid_Parameter;

typedef struct
{
	float integral;
	float lastError;
	float lastInput;
	float output;
	float pValue;
	float iValue;
	float dValue;
}t_pid_Control;

t_pid_Control* f_pid_calculateThrottle(float setPoint, float input, t_pid_Control* Ctrl, t_pid_Parameter* Param);


#endif /* INC_PID_H_ */

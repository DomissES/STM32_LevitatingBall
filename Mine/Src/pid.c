/*
 * pid.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */


#include "pid.h"

t_pid_Control* f_pid_CalculateThrottle(int32_t setPoint, int32_t input, t_pid_Control* Ctrl, t_pid_Parameter* Param)
{
	int32_t error;

	error = setPoint - input;

	//get p value
	Ctrl->pValue = error * Param->Kp;

	//get i value (the fancy way)
	Ctrl->integral += (error + Ctrl->lastError)/2;

	if(Ctrl->integral < Param->I_minRange) Ctrl->integral = Param->I_minRange;
	else if(Ctrl->integral > Param->I_maxRange) Ctrl->integral = Param->I_maxRange;
	Ctrl->iValue = Ctrl->integral * Param->Ki;

	//get d value but from input instead of error
	Ctrl->dValue = Param->Kd*(input - Ctrl->lastInput);

	Ctrl->output = Ctrl->pValue + Ctrl->iValue + Ctrl->dValue;
	Ctrl->lastInput = input;
	Ctrl->lastError = error;

	if(Ctrl->output > Param->maxRange) Ctrl->output = Param->maxRange;
	else if(Ctrl->output < Param->minRange) Ctrl->output = Param->minRange;

	return Ctrl;
}

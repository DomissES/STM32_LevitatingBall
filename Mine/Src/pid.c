/*
 * pid.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */


#include "pid.h"

t_pid_Control* f_pid_calculateThrottle(float setPoint, float input, t_pid_Control* Ctrl, t_pid_Parameter* Param)
{
	float error = setPoint - input;

	//get p value
	Ctrl->pValue = error * Param->Kp;

	//get i value (with little lowPass filter)
	Ctrl->integral += (error + Ctrl->lastError)/2 * Param->Ki;

	Ctrl->iValue = Ctrl->integral;
	if(Ctrl->integral < Param->I_minRange) Ctrl->integral = Param->I_minRange;
	else if(Ctrl->integral > Param->I_maxRange) Ctrl->integral = Param->I_maxRange;


	//get d value but from input instead of error
	Ctrl->dValue = Param->Kd*(input - Ctrl->lastInput);

	Ctrl->output = Ctrl->pValue + Ctrl->iValue + Ctrl->dValue;
	Ctrl->lastInput = input;
	Ctrl->lastError = error;

	if(Ctrl->output > Param->maxRange) Ctrl->output = Param->maxRange;
	else if(Ctrl->output < Param->minRange) Ctrl->output = Param->minRange;

	return Ctrl;
}

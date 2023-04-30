/*
 * runtime.h
 *
 *  Created on: 25 kwi 2023
 *      Author: domis
 */

#ifndef INC_RUNTIME_H_
#define INC_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"


bool f_runtime_FirstInit();
bool f_runtime_FirstTest();
float f_runtime_GetParamInput(uint8_t multiplier); //default resolution is 0.01

// performs sensor measure, calculations and setting desired motor power output, returns 'motorPwm'
uint16_t f_runtime_WorkMotorHandler(uint16_t *distanceGet, uint16_t *distanceSet, uint16_t *motorPwm, t_pid_Parameter *Param, t_pid_Control *Ctrl);

#endif /* INC_RUNTIME_H_ */

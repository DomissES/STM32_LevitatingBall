/*
 * state_machine.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */


#include "state_machine.h"


e_sm_State sm_NextState;
volatile e_sm_Event sm_EventFlag;

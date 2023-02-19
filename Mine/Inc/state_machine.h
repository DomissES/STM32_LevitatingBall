/*
 * state_machine.h
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

typedef enum
{
	st_error,
	st_init,
	st_idle,
	st_work,
	st_test,
}e_sm_State;

typedef enum
{
	ev_error,
	ev_button,
	ev_timeout,
	ev_lcd_refresh,
}e_sm_Event;

typedef e_sm_State (*f_sm_handler)();

typedef struct
{
	e_sm_State state;
	f_sm_handler handler;
}t_smStateMachine;

extern e_sm_State sm_NextState;
extern volatile e_sm_Event sm_EventFlag;

e_sm_State f_sm_GetCurrentState();
e_sm_Event f_sm_ReadEvent();

#endif /* INC_STATE_MACHINE_H_ */

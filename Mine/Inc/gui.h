/*
 * gui.h
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include <stdint.h>

#include "pid.h"
#include "state_machine.h"

typedef const struct
{
	char txtA[16];
	char txtB[16];
	char txtC[16];
} t_gui_ParamTitles;

typedef enum {LCD_ERROR, LCD_INIT, LCD_INPUT, LCD_PARAM, LCD_CTRL, LCD_CHART} e_gui_lcdPage;

void f_gui_DrawChartPage(uint8_t *data, uint8_t length, uint8_t shift);
void f_gui_DrawParamPage(t_pid_Parameter *Param, t_pid_Control *Ctrl);
void f_gui_DrawCtrlPage(float set, float input, float output);
void f_gui_DrawHeading(e_sm_State state, e_gui_lcdPage Page); // state | lcd page

#endif /* INC_GUI_H_ */

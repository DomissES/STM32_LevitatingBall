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
#include "main.h"

typedef enum {LCD_ERROR, LCD_INIT, LCD_INPUT, LCD_PARAM, LCD_CTRL, LCD_CHART, LCD_NOPAGE} e_gui_lcdPage;

// draws chart on LCD. Data should be a max 120 element array. "shift" shifts the data to the right on LCD to make look the data moves.
void f_gui_DrawChartPage(uint8_t *pData, uint8_t length, uint8_t shift);

// draws t_pid_Parameter values on LCD. It look (<Parameter set>) parameter value
void f_gui_DrawParamPage(t_pid_Parameter *Param, t_pid_Control *Ctrl);

// draws distance set, distance measured, motor output on LCD.
void f_gui_DrawCtrlPage(float set, float input, float output);

//draws status of the program in the first line on LCD.
void f_gui_DrawHeading(e_sm_State state, e_gui_lcdPage Page); // state | lcd page

//draws any of the upper functions based on 'page' parameter
void f_gui_DrawPage(e_gui_lcdPage page, t_pid_Parameter *Param, t_pid_Control *Ctrl, uint16_t pwmOutput, uint16_t distanceSet, uint16_t distanceGet);

#endif /* INC_GUI_H_ */

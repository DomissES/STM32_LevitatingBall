/*
 * ui.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#include <string.h>

#include "gui.h"
#include "lcd_service.h"

static const char *StateTitle[] =
{
	"Error",
	"Init",
	"Idle",
	"Work",
	"Test"
};

static const char *PageTitle[] = 
{
	"Error",
	"Init",
	"Input",
	"PID Param",
	"PID Ctrl",
	"Chart"
};


static inline void f_gui_ClearLowerLcdPart()
{
	for(uint8_t i = 2; i < 8; i++) f_lcd_Clear(0, 127, i);
}


//=============== PUBLIC FUNCTIONS ==========================

void f_gui_DrawChart(uint8_t *data, uint8_t length, uint8_t shift)
{
	if(length > 120) return;

	f_gui_ClearLowerLcdPart();

	for(uint8_t i = 0; i < length; i++)
	{
		uint8_t chartValue = 62 - data[(i + shift) % length]; //offset 2px
		if(chartValue < 18) chartValue = 18;
		f_lcd_SetPixel(i + 3, chartValue, true);
	}

	f_lcd_DrawLine(0, 16, 0, 64);
	f_lcd_DrawLine(0, 63, 127, 63);
}

void f_gui_DrawParamPage(t_pid_Parameter *Param, t_pid_Control *Ctrl)
{
	char txt[32];
	f_gui_ClearLowerLcdPart();

	//Parameter: (set) get

	sprintf(txt, "P:\t(%.1f)\t%.1f", Param->Kp, Ctrl->pValue);
	f_lcd_WriteTxt(0, 16, txt, &font_msSansSerif_14);
	sprintf(txt, "I:\t(%.1f)\t%.1f", Param->Ki, Ctrl->iValue);
	f_lcd_WriteTxt(0, 32, txt, &font_msSansSerif_14);
	sprintf(txt, "D:\t(%.1f)\t%.1f", Param->Kd, Ctrl->dValue);
	f_lcd_WriteTxt(0, 48, txt, &font_msSansSerif_14);
}

void f_gui_DrawCtrlPage(float set, float input, float output)
{
	char txt[32];
	f_gui_ClearLowerLcdPart();

	sprintf(txt, "Set:\t%.1f", set);
	f_lcd_WriteTxt(0, 16, txt, &font_msSansSerif_14);
	sprintf(txt, "In:\t\t%.1f", input);
	f_lcd_WriteTxt(0, 32, txt, &font_msSansSerif_14);
	sprintf(txt, "Out:\t%.1f", output);
	f_lcd_WriteTxt(0, 48, txt, &font_msSansSerif_14);
}

void f_gui_DrawHeading(e_sm_State state, e_gui_lcdPage Page)
{
	char txt[32];
	f_lcd_Clear(0, 127, 0);
	f_lcd_Clear(0, 127, 1);
	
	f_lcd_WriteTxt(0, 0, *StateTitle[state], &font_msSansSerif_14);
	f_lcd_WriteTxt(0, 64, *PageTitle[Page], &font_msSansSerif_14);
}

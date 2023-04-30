/*
 * ui.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#include <stdio.h>

#include "gui.h"
#include "lcd_service.h"

static const char * const StateTitle[] =
{
	"Error",
	"Init",
	"Idle",
	"Work",
	"Test",
	"Replay"
};

static const char * const PageTitle[] =
{
	"Error",
	"Init",
	"Input",
	"Param",
	"Ctrl",
	"Chart",
	"---"
};


static inline void f_gui_ClearLowerLcdPart()
{
	for(uint8_t i = 2; i < 8; i++) f_lcd_Clear(0, 128, i);
}


//=============== PUBLIC FUNCTIONS ==========================

void f_gui_DrawChartPage(uint8_t *pData, uint8_t length, uint8_t shift)
{
	//max length: 120 px
	//max value: 44 px
	if(length > 120) return;

	for(uint8_t i = 0; i < length; i++)
	{
		uint8_t chartValue = 62 - pData[(i + shift) % length]; //offset 2px and reversed upside down
		if(chartValue < 18) chartValue = 18;
		f_lcd_SetPixel(i + 3, chartValue, true);
	}

	f_lcd_DrawLine(0, 16, 0, 64);
	f_lcd_DrawLine(0, 63, 127, 63);
}

void f_gui_DrawParamPage(t_pid_Parameter *Param, t_pid_Control *Ctrl)
{
	char txt[32];

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

	sprintf(txt, "Set:\t%.1f cm", set);
	f_lcd_WriteTxt(0, 16, txt, &font_msSansSerif_14);
	sprintf(txt, "In:\t\t%.1f cm", input);
	f_lcd_WriteTxt(0, 32, txt, &font_msSansSerif_14);
	sprintf(txt, "Out:\t%.1f", output);
	f_lcd_WriteTxt(0, 48, txt, &font_msSansSerif_14);
}

void f_gui_DrawHeading(e_sm_State state, e_gui_lcdPage Page)
{
	f_lcd_Clear(0, 128, 0);
	f_lcd_Clear(0, 128, 1);
	
	f_lcd_WriteTxt(0, 0, "St:", &font_msSansSerif_14);
	f_lcd_WriteTxt(24, 0, StateTitle[state], &font_msSansSerif_14);
	f_lcd_WriteTxt(64, 0, "Lcd:", &font_msSansSerif_14);
	f_lcd_WriteTxt(96, 0, PageTitle[Page], &font_msSansSerif_14);
}

void f_gui_DrawPage(e_gui_lcdPage page, t_pid_Parameter *Param, t_pid_Control *Ctrl, uint16_t pwmOutput, uint16_t distanceSet, uint16_t distanceGet)
{
	static uint8_t chartData[120];
	static uint8_t chartIterator, chartLength;

	f_gui_ClearLowerLcdPart();

	switch (page)
	{
		case LCD_PARAM:
			f_gui_DrawParamPage(Param, Ctrl);
			break;

		case LCD_CTRL:
			f_gui_DrawCtrlPage((float)distanceSet/10, (float)distanceGet/10, (float)pwmOutput/41);
			break;

		case LCD_CHART:
			chartData[chartIterator] = (uint32_t)(pwmOutput*44)/4096; //max value is 44px
			chartIterator = (chartIterator + 1) % 120; // it should be here, so the newest sample is not at the beginning

			if(chartLength < 120)
			{
				f_gui_DrawChartPage(chartData, chartLength, 0);
				chartLength++;
			}
			else
			{
				f_gui_DrawChartPage(chartData, chartLength, chartIterator);
			}

			break;

		default:
			f_lcd_ClearAll();
			break;
	}
}


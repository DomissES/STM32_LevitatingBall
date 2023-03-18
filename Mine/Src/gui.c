/*
 * ui.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#include "gui.h"
#include "lcd_service.h"




void f_gui_drawChart(uint8_t *data, uint8_t length, uint8_t shift)
{
	if(length > 120) return;

	for(uint8_t i = 2; i < 8; i++) f_lcd_Clear(0, 127, i);

	for(uint8_t i = 0; i < length; i++)
	{
		uint8_t chartValue = 62 - data[(i + shift) % length];
		if(chartValue < 18) chartValue = 18;
		f_lcd_SetPixel(i + 3, chartValue, true);
	}

	f_lcd_DrawLine(0, 16, 0, 64);
	f_lcd_DrawLine(0, 63, 127, 63);
}

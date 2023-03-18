/*
 * gui.h
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include <stdint.h>

typedef const struct
{
	char txtA[16];
	char txtB[16];
	char txtC[16];
} t_gui_ParamTitles;

void f_gui_drawChartPage(uint8_t *data, uint8_t length, uint8_t shift);
void f_gui_drawParam();

#endif /* INC_GUI_H_ */

/*
 * ssd1331_hw.h
 *
 *  Created on: 8 sty 2023
 *      Author: domis
 */

#ifndef INC_SH1106_HW_H_
#define INC_SH1106_HW_H_

#include <stdbool.h>
#include <stdint.h>

#define SH1106_WIDTH	128
#define SH1106_HEIGHT	64


void f_sh1106_Init();

bool f_sh1106_cmd_SetColumnAddress(uint8_t address);
bool f_sh1106_cmd_SetStartLine(uint8_t line);
bool f_sh1106_cmd_SetContrast(uint8_t cont);
bool f_sh1106_cmd_SetSegmentDirection(bool reverse);
bool f_sh1106_cmd_ToggleDisplay(bool on);
bool f_sh1106_cmd_ToggleNegative(bool reverse);
bool f_sh1106_cmd_SetMultiplexRatio(uint8_t mux);
bool f_sh1106_cmd_TogglePower(bool sleep);
bool f_sh1106_cmd_SetPageAddress(uint8_t page);
bool f_sh1106_cmd_SetScanDirection(bool reverse);
bool f_sh1106_cmd_SetOffset(uint8_t offset);
bool f_sh1106_cmd_SetClockDiv(uint8_t divide, uint8_t freq);

void f_sh1106_CS_HIGH();
bool f_sh1106_SendPageData(uint8_t page, uint8_t *data, uint8_t length);

#endif /* INC_SH1106_HW_H_ */

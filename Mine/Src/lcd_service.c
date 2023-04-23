/*
 * lcd_service.c
 *
 *  Created on: 8 sty 2023
 *      Author: domis
 */


#include "lcd_service.h"
#include "sh1106_hw.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdfix.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include <math.h>

uint8_t sh1106_FrameBuffer[SH1106_HEIGHT/8][SH1106_WIDTH];
uint8_t sh1106_pageDirtyMask;


static inline void f_SendFrameBufferPage(uint8_t page)
{
	f_sh1106_cmd_SetColumnAddress(0);
	f_sh1106_cmd_SetPageAddress(page);
	f_sh1106_SendPageData(page, &sh1106_FrameBuffer[page][0], SH1106_WIDTH);
}

//=========== public functions =================

void f_lcd_Init()
{
	f_sh1106_Init();

	f_sh1106_cmd_SetScanDirection(1);
	f_sh1106_cmd_SetSegmentDirection(1);
	f_lcd_ClearAll();

	for(uint8_t page = 0; page < 8; page++)
	{
		f_SendFrameBufferPage(page);
		HAL_Delay(4);
	}

	f_sh1106_cmd_TogglePower(1);

}

void f_lcd_Clear(uint8_t col_start, uint8_t col_end, uint8_t page)
{
	uint8_t blank = 0x00;

	for(uint8_t i = col_start; i < col_end; i++)
	{
		sh1106_FrameBuffer[page][i] = blank;
	}

	sh1106_pageDirtyMask |= (1 << page);
}

void f_lcd_ClearAll()
{
	for(uint8_t page = 0; page < 8; page++) f_lcd_Clear(0, SH1106_WIDTH, page);
}

void f_lcd_WriteTxt(uint8_t x, uint8_t y, const char* txt, const tFont *font)
{
	uint8_t fontHeight = font->chars->image->height;
	uint8_t ch;

	while((ch = *txt++))
	{
		if(ch != '\t')
		{
			uint8_t fontWidth = font->chars[ch - 0x20].image->width;
			if((SH1106_WIDTH - x) < fontWidth) break; //no line wrapping
			f_lcd_DrawRaw(x, y, font->chars[ch - 0x20].image->data, fontWidth, fontHeight);
	
			x += fontWidth;
		}
		else //tab
		{
			x = ((x/16) + 1) * 16;
		}
		
	}
}

void f_lcd_DrawRaw(uint8_t x, uint8_t y, const uint8_t* image, uint8_t x_size, uint8_t y_size)
{
	uint8_t bufferPage;
	uint8_t imagePage = 0;

	uint8_t bitsLeftInBuffer;
	uint8_t bitsLeftInImage = 8; //first row of image is always full byte
	uint8_t bitsToWrite;

	bool writtenUpperHalf; //if false then lower half


	while(y_size)
	{
		bufferPage = y/8;
		bitsLeftInBuffer = 8 - (y & 0x07);

		if(bitsLeftInImage > bitsLeftInBuffer) //probably you write on lower half (upper bits)
			{
				bitsToWrite = bitsLeftInBuffer;
				writtenUpperHalf = false;
			}
		else
			{
				bitsToWrite = bitsLeftInImage;
				writtenUpperHalf = true;
			}

		for(uint8_t lines = 0; lines < x_size; lines++)
		{
			uint8_t dataToPreserve;
			uint8_t dataToWrite;

			if(writtenUpperHalf) //preserve MSB bits
			{
				dataToPreserve = sh1106_FrameBuffer[bufferPage][x + lines] & (0xFF << bitsToWrite);
				dataToWrite = image[imagePage*x_size + lines] >> (8 - bitsToWrite);
			}
			else //preserve LSB bits
			{
				dataToPreserve = sh1106_FrameBuffer[bufferPage][x + lines] & (0xFF >> bitsToWrite);
				dataToWrite = image[imagePage*x_size + lines] << (8 - bitsToWrite);
			}

			sh1106_FrameBuffer[bufferPage][x + lines] = dataToPreserve | dataToWrite;//write image to page
		}

		bitsLeftInImage -= bitsToWrite;
		if(bitsLeftInImage == 0) //jump to next page from image
		{
			imagePage++;
			bitsLeftInImage = 8;
		}

		y_size -= bitsToWrite;
		y += bitsToWrite;

		sh1106_pageDirtyMask |= (1 << bufferPage);

	}

}

void f_lcd_DrawImage(uint8_t x, uint8_t y, const tImage* image)
{
	f_lcd_DrawRaw(x, y, image->data, image->width, image->height);
}

void f_lcd_SetPixel(uint8_t x, uint8_t y, bool set)
{
	if(set) sh1106_FrameBuffer[y/8][x] |= (1 << (y&0x07));
	else sh1106_FrameBuffer[y/8][x] &= ~(1 << (y&0x07));
}

void f_lcd_DrawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd)
{
	float delx, dely, length;
	float x, y;

	delx = abs(xEnd - xStart);
	dely = abs(yEnd - yStart);

	if(delx < dely) length = dely;
	else length = delx;

	delx = (xEnd - xStart) / length;
	dely = (yEnd - yStart) / length;

	x = xStart + 0.5;
	y = yStart + 0.5;

	for(uint8_t i = 0; i <= length; i ++)
	{
		f_lcd_SetPixel((uint8_t)x, (uint8_t)y, 1);

		x += delx;
		y += dely;
	}

	for(uint8_t i = yStart/8; i <= yEnd/8; i++) sh1106_pageDirtyMask |= (1 << i);

	//check out for bresenham algorithm to optimize for bigger screens
}


void f_lcd_SendFrameCallback()
{
	static uint8_t timer;
	static uint8_t activePage;


	if((HAL_GetTick() - timer) > 5) //software timer
	{

		if(sh1106_pageDirtyMask & (1 << activePage))
		{
			f_SendFrameBufferPage(activePage);

			sh1106_pageDirtyMask &= ~(1 << activePage);

		}

		activePage = (activePage + 1) % 8;
		timer = HAL_GetTick();
	}
}


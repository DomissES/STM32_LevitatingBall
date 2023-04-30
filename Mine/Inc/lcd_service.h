/*
 * lcd_service.h
 *
 *  Created on: 8 sty 2023
 *      Author: domis
 */

#ifndef INC_LCD_SERVICE_H_
#define INC_LCD_SERVICE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
     const uint8_t *data;
     uint16_t width;
     uint16_t height;
     uint8_t dataSize;
     } tImage;
 typedef struct {
     long int code;
     const tImage *image;
     } tChar;
 typedef struct {
     int length;
     const tChar *chars;
     } tFont;

extern const tFont font_msSansSerif_14;

bool f_lcd_Init();
void f_lcd_Clear(uint8_t colStart, uint8_t colEnd, uint8_t page);
void f_lcd_ClearAll();
void f_lcd_WriteTxt(uint8_t x, uint8_t y, const char* txt, const tFont *pFont);

//draw raw pixels from pData. Refer to sh1106 datasheet how memory is mapped
void f_lcd_DrawRaw(uint8_t x, uint8_t y, const uint8_t* pData, uint8_t xSize, uint8_t ySize);
void f_lcd_DrawImage(uint8_t x, uint8_t y, const tImage* pImage);
void f_lcd_SetPixel(uint8_t x, uint8_t y, bool set);
void f_lcd_DrawLine(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd);

// has to be called at regular intervals, send via DMA data to lcd
void f_lcd_SendFrameCallback();


#endif /* INC_LCD_SERVICE_H_ */

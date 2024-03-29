/*
 * ssd1331_hw.c
 *
 *  Created on: 8 sty 2023
 *      Author: domis
 */

#include "sh1106_hw.h"
#include "spi.h"
#include "main.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx_hal.h>


#define GPIO_HIGH(port, pin)	(port)->BSRR = (pin)
#define GPIO_LOW(port, pin)		(port)->BSRR = ((pin) << 16)

// Chip select
#define CS_HIGH()		GPIO_HIGH(SPI2_CS_GPIO_Port, SPI2_CS_Pin)
#define CS_LOW()		GPIO_LOW(SPI2_CS_GPIO_Port, SPI2_CS_Pin)
// Command
#define CMD_HIGH()		GPIO_HIGH(LCD_CMD_GPIO_Port, LCD_CMD_Pin)
#define CMD_LOW()		GPIO_LOW(LCD_CMD_GPIO_Port, LCD_CMD_Pin)
// Reset
#define RES_HIGH()		GPIO_HIGH(LCD_RES_GPIO_Port, LCD_RES_Pin)
#define RES_LOW()		GPIO_LOW(LCD_RES_GPIO_Port, LCD_RES_Pin)

bool sh1106_TransmitBusy;
SPI_HandleTypeDef *pSPIhandler; //local handler

static bool f_sh1106_HwInit(SPI_HandleTypeDef *pHandler, SPI_TypeDef *pSPIaddress)
{
	pSPIhandler = pHandler; //assign to a local handler

	pHandler->Instance = pSPIaddress;
	pHandler->Init.Mode = SPI_MODE_MASTER;
	pHandler->Init.Direction = SPI_DIRECTION_2LINES;
	pHandler->Init.DataSize = SPI_DATASIZE_8BIT;
	pHandler->Init.CLKPolarity = SPI_POLARITY_HIGH;
	pHandler->Init.CLKPhase = SPI_PHASE_2EDGE;
	pHandler->Init.NSS = SPI_NSS_SOFT;
	pHandler->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	pHandler->Init.FirstBit = SPI_FIRSTBIT_MSB;
	pHandler->Init.TIMode = SPI_TIMODE_DISABLE;
	pHandler->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	pHandler->Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(pHandler) != HAL_OK)
	{
		Error_Handler();
		return false;
	}

	__HAL_SPI_ENABLE(pSPIhandler);
	CS_HIGH();

	return true;
}

static bool f_sh1106_TrySendData(uint8_t *pData, uint16_t length, bool cmd)
{
	bool isOk = false;
	uint8_t tries;

	for(tries = 0; tries < 3; tries++)
	{
		if(sh1106_TransmitBusy)
		{
			HAL_Delay(10);
		}
		else
		{
			if(cmd) CMD_LOW();
			else CMD_HIGH();

			sh1106_TransmitBusy = true;
			CS_LOW();
			HAL_StatusTypeDef status = HAL_SPI_Transmit(pSPIhandler, pData, length, 10);
			CS_HIGH();
			sh1106_TransmitBusy = false;

			if(status == HAL_OK) isOk = true;

			break;
		}
	}

	return isOk;
}

//============ public functions ========================

bool f_sh1106_Init(SPI_HandleTypeDef *pHandler, SPI_TypeDef *pSPIaddress)
{
	bool isOk;

	if(pHandler == NULL) return false;

	isOk = f_sh1106_HwInit(pHandler, pSPIaddress);

	RES_LOW();
	HAL_Delay(1);
	RES_HIGH();

	return isOk;
}

// ========================== COMMAND SEND FUNCTIONS ====================

bool f_sh1106_cmd_SetColumnAddress(uint8_t address)
{
	if(address >= SH1106_WIDTH) return false;

	address += 2; //2 first and last pixels are not visible
	uint8_t cmd[2];

	cmd[0] = 0x10 | ((address >> 4) & 0x0F);
	cmd[1] = address & 0x0F;

	bool isOk = f_sh1106_TrySendData((uint8_t*)&cmd, 2, true);

	return isOk;
}

bool f_sh1106_cmd_SetStartLine(uint8_t line)
{
	if(line >= SH1106_HEIGHT) return false;

	uint8_t cmd = 0x40 | (line & 0x3F);

	bool isOk = f_sh1106_TrySendData(&cmd, 1, true);

	return isOk;
}

bool f_sh1106_cmd_SetContrast(uint8_t cont)
{
	uint8_t cmd[2];

	cmd[0] = 0x81;
	cmd[1] = cont;

	bool isOk = f_sh1106_TrySendData((uint8_t*)&cmd, 2, true);

	return isOk;
}

bool f_sh1106_cmd_SetSegmentDirection(bool reverse)
{
	uint8_t cmd = 0xA0 | (reverse & 0x01);

	bool isOk = f_sh1106_TrySendData(&cmd, 1, true);

	return isOk;
}

bool f_sh1106_cmd_ToggleDisplay(bool on)
{
	uint8_t cmd = 0xA4 | (on & 0x01);

	bool isOk = f_sh1106_TrySendData(&cmd, 1, true);

	return isOk;
}

bool f_sh1106_cmd_ToggleNegative(bool reverse)
{
	uint8_t cmd = 0xA6 | (reverse & 0x01);

	bool isOk = f_sh1106_TrySendData(&cmd, 1, true);

	return isOk;
}

bool f_sh1106_cmd_SetMultiplexRatio(uint8_t mux)
{
	if(mux > 63) return false;

	uint8_t cmd[2];

	cmd[0] = 0xA8;
	cmd[1] = mux;

	bool isOk = f_sh1106_TrySendData((uint8_t*)&cmd, 2, true);

	return isOk;
}

bool f_sh1106_cmd_TogglePower(bool sleep)
{
	uint8_t cmd = 0xAE | (sleep & 0x01);

	bool isOk = f_sh1106_TrySendData(&cmd, 1, true);

	return isOk;
}

bool f_sh1106_cmd_SetPageAddress(uint8_t page)
{
	if(page >= SH1106_HEIGHT/8) return false;

	uint8_t cmd = 0xB0 | (page & 0x0F);

	bool isOk = f_sh1106_TrySendData(&cmd, 1, true);

	return isOk;
}

bool f_sh1106_cmd_SetScanDirection(bool reverse)
{
	uint8_t cmd = 0xC0 | ((reverse << 3) & 0x0F);

	bool isOk = f_sh1106_TrySendData(&cmd, 1, true);

	return isOk;
}

bool f_sh1106_cmd_SetOffset(uint8_t offset)
{
	if(offset >= SH1106_HEIGHT) return false;

	uint8_t cmd[2];

	cmd[0] = 0xD3;
	cmd[1] = offset;

	bool isOk = f_sh1106_TrySendData((uint8_t*)&cmd, 2, true);

	return isOk;
}

bool f_sh1106_cmd_SetClockDiv(uint8_t divide, uint8_t freq)
{
	if((divide > 15) && (freq > 15)) return false;

	uint8_t cmd[2];

	cmd[0] = 0xD5;
	cmd[1] = ((divide << 4) & 0xF0) | (freq & 0x0F);

	bool isOk = f_sh1106_TrySendData((uint8_t*)&cmd, 2, true);

	return isOk;
}

bool f_sh1106_cmd_ReadStatus(uint8_t *pStatus)
{
	bool isOk = false;
	uint8_t dummy = 0x00;

	if(sh1106_TransmitBusy) isOk = false;
	else
	{
		CMD_LOW();
		sh1106_TransmitBusy = true;
		CS_LOW();
		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(pSPIhandler, &dummy, pStatus, 1, 10);
		CS_HIGH();
		sh1106_TransmitBusy = false;

		if(status == HAL_OK) isOk = true;
	}

	return isOk;
}
// =============================

void f_sh1106_CS_HIGH()
{
	CS_HIGH();
}

bool f_sh1106_SendDataDMA(uint8_t *pData, uint8_t length)
{
	uint8_t tries = 0;

	HAL_StatusTypeDef status;
	do
	{
		CS_LOW();
		CMD_HIGH(); //only data is send, no need to make it a variable;;
		status = HAL_SPI_Transmit_DMA(pSPIhandler, pData, length);
		tries++;
	}while((status != HAL_OK) && tries < 5);

	if(tries == 5)
	{
		CS_HIGH();
		return false;
	}

	sh1106_TransmitBusy = true;
	return true;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi == pSPIhandler)
	{
		f_sh1106_CS_HIGH();
		sh1106_TransmitBusy = false;
	}
}


/*
 * ina219.c
 *
 *  Created on: Jan 23, 2023
 *      Author: domis
 */

//URL to the datasheet
// https://www.ti.com/lit/ds/symlink/ina219.pdf

#include <stm32f4xx_hal.h>
#include <stdfix.h>
#include "i2c.h"
#include "ina219.h"

//refer to the datasheet
#define INA219_CURRENT_LSB		(INA219_EXPECTED_MAX_CURRENT/32768)
#define INA219_CALIB_VALUE		(0.04096/(INA219_CURRENT_LSB * INA219_SHUNT_RESISTANCE))

#define INA219_I2C_ADDRESS		0x80

#define INA219_CONFIG			0x00
#define INA219_SHUNT_VOLTAGE	0x01
#define INA219_BUS_VOLTAGE		0x02
#define INA219_POWER			0x03
#define INA219_CURRENT			0x04
#define INA219_CALIBRATION		0x05

uint16_t curr_reg = INA219_CALIB_VALUE;
uint16_t curr_lsc = INA219_CURRENT_LSB * 1000000;

I2C_HandleTypeDef *pI2Chandler; //local i2c handler

static uint16_t f_receiveWord(uint8_t address)
{
	uint8_t receive[2];

	HAL_I2C_Master_Transmit(pI2Chandler, INA219_I2C_ADDRESS, &address, 1, 50);
	HAL_I2C_Master_Receive(pI2Chandler, INA219_I2C_ADDRESS, (uint8_t*)&receive, 2, 50);

	return (receive[0] << 8) | receive[1];
}

static void f_sendWord(uint8_t address, uint16_t word)
{
	uint8_t transmit[3] = {address, word >> 8, word & 0xFF};

	HAL_I2C_Master_Transmit(pI2Chandler, INA219_I2C_ADDRESS, (uint8_t*)&transmit, 3, 50);
}

static inline void f_ina219_HwInit(I2C_HandleTypeDef *pHandler, I2C_TypeDef *pI2Caddress)
{
	pI2Chandler = pHandler; //assign local i2c handler

	pHandler->Instance = pI2Caddress;
	pHandler->Init.ClockSpeed = 100000;
	pHandler->Init.DutyCycle = I2C_DUTYCYCLE_2;
	pHandler->Init.OwnAddress1 = 0;
	pHandler->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	pHandler->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	pHandler->Init.OwnAddress2 = 0;
	pHandler->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	pHandler->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(pHandler) != HAL_OK)
	{
	   Error_Handler();
	}
}

static inline void f_ina219_calibrate()
{
	uint16_t calib = (uint16_t)INA219_CALIB_VALUE;

	f_sendWord(INA219_CALIBRATION, calib);
}

//============== public functions ==================

bool f_ina219_Init(I2C_HandleTypeDef *pHandler, I2C_TypeDef *pI2Caddress)
{
	bool isOk = false;

	if(pHandler == NULL) return false;
	f_ina219_HwInit(pHandler, pI2Caddress);

	//set the config register
	// bus voltage range = 32v
	// gain divide = 2
	// bus mode/average = 12bit/
	// shunt mode/average = 12bit/ 128
	//mode = bus and shunt, continuous
	uint16_t config = (1 << 13) | (1 << 11) | (15 << 7) | (15 << 3) | (7 << 0);

	f_ina219_Reset();
	HAL_Delay(1);
	f_sendWord(INA219_CONFIG, config);

	//check if sensor is connected
	uint16_t checkConfig = f_receiveWord(INA219_CONFIG);
	if(checkConfig == config) isOk = true;

	f_ina219_calibrate();

	return isOk;
}

void f_ina219_Reset()
{
	uint16_t tempConfig = (1 << 15);

	f_sendWord(INA219_CONFIG, tempConfig);
}

void f_ina219_Mode(e_ina219_PowerMode Mode)
{
	uint16_t tempConfig;

	tempConfig = f_receiveWord(INA219_CONFIG);

	tempConfig = (tempConfig & 0xFFF8) | (uint8_t)Mode;
	HAL_I2C_Master_Transmit(pI2Chandler, INA219_I2C_ADDRESS, (uint8_t*)&tempConfig, 2, 50);
}

int16_t f_ina219_GetCurrentInMilis()
{
	int16_t current = f_receiveWord(INA219_CURRENT);

	return 1000*current*INA219_CURRENT_LSB;
}

uint16_t f_ina219_GetPowerInMilis()
{
	uint16_t power = f_receiveWord(INA219_POWER);

	return 1000*20*power*INA219_CURRENT_LSB;
}

int16_t f_ina219_GetBusVoltageInMilis()
{
	int16_t voltage = f_receiveWord(INA219_BUS_VOLTAGE);
	voltage = 4*(voltage >> 3);

	return voltage;
}

int16_t f_ina219_GetShuntVoltageInMicro()
{
	int16_t voltage = f_receiveWord(INA219_SHUNT_VOLTAGE);

	return 10*voltage;
}

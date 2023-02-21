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


static uint16_t f_receiveWord(uint8_t address)
{
	uint8_t receive[2];

	HAL_I2C_Master_Transmit(&hi2c1, INA219_I2C_ADDRESS, &address, 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, INA219_I2C_ADDRESS, (uint8_t*)&receive, 2, 50);

	return (receive[0] << 8) | receive[1];
}

static inline void f_sendWord(uint8_t address, uint16_t word)
{
	uint8_t transmit[3] = {address, word >> 8, word & 0xFF};

	HAL_I2C_Master_Transmit(&hi2c1, INA219_I2C_ADDRESS, (uint8_t*)&transmit, 3, 50);
}

static void f_ina219_HwInit()
{
	 hi2c1.Instance = I2C1;
	 hi2c1.Init.ClockSpeed = 100000;
	 hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	 hi2c1.Init.OwnAddress1 = 0;
	 hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	 hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	 hi2c1.Init.OwnAddress2 = 0;
	 hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	 hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	 if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	 {
	   Error_Handler();
	 }
}

static void f_ina219_calibrate()
{
	uint16_t calib = (uint16_t)INA219_CALIB_VALUE;

	f_sendWord(INA219_CALIBRATION, calib);
}

//============== public functions ==================

void f_ina219_Init()
{
	f_ina219_HwInit();

	//set the config register
	// bus voltage range = 32v
	// gain divide = 2
	// bus mode/average = 12bit/
	// shunt mode/average = 12bit/ 128
	//mode = bus and shunt, continuous
	uint16_t config = (1 << 13) | (1 << 11) | (15 << 7) | (15 << 3) | (7 << 0);

	f_ina219_reset();
	HAL_Delay(1);
	f_sendWord(INA219_CONFIG, config);

	f_ina219_calibrate();

}

void f_ina219_reset()
{
	uint16_t tempConfig = (1 << 15);

	f_sendWord(INA219_CONFIG, tempConfig);
}

void f_ina219_Mode(uint8_t mode)
{
	uint16_t tempConfig;

	tempConfig = f_receiveWord(INA219_CONFIG);

	tempConfig = (tempConfig & 0xFFF8) | mode;
	HAL_I2C_Master_Transmit(&hi2c1, INA219_I2C_ADDRESS, (uint8_t*)&tempConfig, 2, 50);
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

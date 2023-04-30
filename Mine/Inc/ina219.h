/*
 * ina219.h
 *
 *  Created on: Jan 23, 2023
 *      Author: domis
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include <stdint.h>
#include <stdfix.h>
#include <stdbool.h>
#include <i2c.h>

#define INA219_EXPECTED_MAX_CURRENT 4.096
#define INA219_SHUNT_RESISTANCE		0.01

typedef enum {OFF, SHUNT_TRIG, BUS_TRIG, SHUNT_BUS_TRIG, ADC_OFF, SHUNT_CNT, BUS_CNT, SHUNT_BUS_CNT} e_ina219_PowerMode;

//returns false if it fails.
bool f_ina219_Init(I2C_HandleTypeDef *pHandler, I2C_TypeDef *pI2Caddress);
void f_ina219_Reset();

void f_ina219_Mode(e_ina219_PowerMode Mode);
int16_t f_ina219_GetCurrentInMilis();
uint16_t f_ina219_GetPowerInMilis();
int16_t f_ina219_GetBusVoltageInMilis();
int16_t f_ina219_GetShuntVoltageInMicro();



#endif /* INC_INA219_H_ */

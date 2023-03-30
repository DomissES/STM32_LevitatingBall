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

#define INA219_EXPECTED_MAX_CURRENT 4.096
#define INA219_SHUNT_RESISTANCE		0.01

#define INA219_MODE_POWERDOWN		0x00
#define INA219_MODE_SHUNT_TRIG		0x01
#define INA219_MODE_BUS_TRIG		0x02
#define INA219_MODE_SHUNT_BUS_TRIG	0x03

#define INA219_MODE_ADC_OFF			0x04
#define INA219_MODE_SHUNT_CONT		0x05
#define INA219_MODE_BUS_CONT		0x06
#define INA219_MODE_SHUNT_BUS_CONT	0x07


bool f_ina219_Init();
void f_ina219_reset();
void f_ina219_Mode(uint8_t mode);
int16_t f_ina219_GetCurrentInMilis();
uint16_t f_ina219_GetPowerInMilis();
int16_t f_ina219_GetBusVoltageInMilis();
int16_t f_ina219_GetShuntVoltageInMicro();



#endif /* INC_INA219_H_ */

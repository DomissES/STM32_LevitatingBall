/*
 * work.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#include "work.h"
#include "tim.h"
#include "ina219.h"

uint8_t velocityOCR;

void f_work_motor_InitTimer()
{
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 167;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 100;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
	{
	Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim9);


}

void f_work_motorSet(bool onOff)
{
	if(onOff) HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	else HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
}

void f_work_motorSetVelocity(uint8_t velocity)
{
	if(velocity >= 100) velocity = 100;
	velocityOCR = velocity;

	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, velocity);
}

bool f_work_motorTest()
{
	bool isBusOk = false;
	bool isPowerOk = false;

	uint8_t busVoltageLow = MOTOR_BUS_VOLTAGE - MOTOR_BUS_VOLTAGE_DEV;
	uint8_t busVoltageHigh = MOTOR_BUS_VOLTAGE + MOTOR_BUS_VOLTAGE_DEV;
	uint16_t busVoltage = f_ina219_GetBusVoltageInMilis()/1000;

	// is there bus voltage?
	if((busVoltage >= busVoltageLow) && (busVoltage <= busVoltageHigh)) isBusOk = true;


	// power draw is within the limit? Units: miliWatts
	uint16_t motorResistance = 1000*(MOTOR_RESISTANCE_K1 * (MOTOR_BUS_VOLTAGE * velocityOCR/100) + MOTOR_RESISTANCE_K2); //in miliOhms
	uint16_t powerHigh = 1000000 * MOTOR_BUS_VOLTAGE * MOTOR_BUS_VOLTAGE / (motorResistance - 1000*MOTOR_RESISTANCE_DEV);
	uint16_t powerLow = MOTOR_BUS_VOLTAGE * MOTOR_BUS_VOLTAGE / (motorResistance + 1000*MOTOR_RESISTANCE_DEV);
	uint16_t power = f_ina219_GetPowerInMilis();

	if((power >= powerLow) && (power <= powerHigh)) isPowerOk = true;

	return isBusOk && isPowerOk;
}

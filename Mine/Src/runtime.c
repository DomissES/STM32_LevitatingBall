/*
 * runtime.c
 *
 *  Created on: 25 kwi 2023
 *      Author: domis
 */


#include "runtime.h"
#include "main.h"
#include "tim.h"
#include "i2c.h"
#include "iwdg.h"

#include "machine.h"
#include "lcd_service.h"
#include "ina219.h"
#include "pid.h"
#include <stdbool.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>


bool f_runtime_FirstInit()
{
	uint8_t isOkCounter = 0;

	//enable all basic peripherals
	isOkCounter += f_lcd_Init();
	f_machine_MotorInitTimer(&htim9, TIM9);
	f_machine_SensorInitTimer(&htim10, TIM10);
	isOkCounter += f_ina219_Init(&hi2c1, I2C1);
	//enable encoder timer
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

	return (isOkCounter == 2) ? true : false;
}

bool f_runtime_FirstTest()
{
	uint8_t isOkCounter = 0;

	//check motor on idle
	f_machine_MotorSet(0);
	isOkCounter += f_machine_MotorTestIfOk();
	//check motor on 50%
	f_machine_MotorSetVelocity(MAX_MOTOR_PWM/2);
	f_machine_MotorSet(1);
	HAL_Delay(2000);
	isOkCounter += f_machine_MotorTestIfOk();
	f_machine_MotorSet(0);

	//check if distance is not 0
	f_machine_SensorTriggerMeasure();
	while(!work_sensorMeasureDone)
		;
	uint16_t distance = f_machine_SensorGetLastMeasure();
	if(distance > 0) isOkCounter++;

	return (isOkCounter == 3) ? true : false;
}

float f_runtime_GetParamInput(uint8_t multiplier) //default resolution is 0.01
{
	char txt[20];
	uint16_t *timerCounter = (uint16_t*)&htim3.Instance->CNT;
	*timerCounter = 0;
	encoderInputChanged = true; //write to lcd the first time, then wait for change

	while(!eventFlag)
	{
		if(encoderInputChanged)
		{
			uint16_t tmpVal = *timerCounter/2;
			sprintf(txt, "%2d.%02d", multiplier*tmpVal/100, multiplier*tmpVal%100);
			f_lcd_Clear(0, 128, 6);
			f_lcd_Clear(0, 128, 7);
			f_lcd_WriteTxt(0, 48, txt, &font_msSansSerif_14);
			encoderInputChanged = false;
		}
		HAL_IWDG_Refresh(&hiwdg); //it stays in this loop quite long, so it is necessary to refresh the watchdog
	}
	return (float)multiplier*(*timerCounter)/200;
}

uint16_t f_runtime_WorkMotorHandler(uint16_t *distanceGet, uint16_t *distanceSet, uint16_t *motorPwm, t_pid_Parameter *Param, t_pid_Control *Ctrl)
{
	uint16_t timeout = 20;

	while(!work_sensorMeasureDone && timeout)
	{
		HAL_Delay(1);
		timeout--;
	}
	*distanceGet = f_machine_SensorGetLastMeasure(); //in mm

	f_pid_calculateThrottle(*distanceSet, (float)*distanceGet, Ctrl, Param);

	*motorPwm = MAX_MOTOR_PWM/2 - (int16_t)Ctrl->output; //error is reversed
	if(*motorPwm > MAX_MOTOR_PWM) *motorPwm = MAX_MOTOR_PWM;
	else if(*motorPwm < 0) *motorPwm = 0;

	f_machine_MotorSetVelocity(*motorPwm);

	f_machine_SensorTriggerMeasure();

	return *motorPwm;
}


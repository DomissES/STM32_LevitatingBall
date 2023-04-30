/*
 * work.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#include <machine.h>
#include "tim.h"
#include "ina219.h"

#define SENSOR_MAX_DISTANCE	2200
#define SENSOR_CORRECTION	5
#define SOUND_VELOCITY		343

uint16_t motorIdlePower;
uint16_t motorVelocityOCR;
uint16_t sensorTimeElapsed;
bool work_sensorMeasureDone; //global variable
bool isMotorWorking;
enum {SENSOR_OFF, SENSOR_TRIGGER, SENSOR_MEASURE, SENSOR_READ} sensorStatus;

TIM_HandleTypeDef *pSensorTIMHandler; //local handler
TIM_HandleTypeDef *pMotorTIMHandler;


// ====================== motor section ====================

void f_machine_MotorInitTimer(TIM_HandleTypeDef *pTIMHandler, TIM_TypeDef *pTIMaddress)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	pMotorTIMHandler = pTIMHandler; //assign to a local pointer

	pTIMHandler->Instance = pTIMaddress;
	pTIMHandler->Init.Prescaler = 4;
	pTIMHandler->Init.CounterMode = TIM_COUNTERMODE_UP;
	pTIMHandler->Init.Period = MAX_MOTOR_PWM;
	pTIMHandler->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pTIMHandler->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(pTIMHandler) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(pTIMHandler, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(pTIMHandler);
}

void f_machine_MotorSet(bool onOff)
{
	if(onOff) HAL_TIM_PWM_Start(pMotorTIMHandler, TIM_CHANNEL_1);
	else HAL_TIM_PWM_Stop(pMotorTIMHandler, TIM_CHANNEL_1);

	isMotorWorking = onOff;
}

void f_machine_MotorSetVelocity(uint16_t velocity)
{
	if(velocity >= MAX_MOTOR_PWM) velocity = MAX_MOTOR_PWM;
	motorVelocityOCR = velocity;

	__HAL_TIM_SET_COMPARE(pMotorTIMHandler, TIM_CHANNEL_1, velocity);
}

bool f_machine_MotorTestIfOk()
{
	bool isBusOk = false;
	bool isPowerOk = false;

	uint16_t busVoltageLow = 1000*(MOTOR_BUS_VOLTAGE - MOTOR_BUS_VOLTAGE_DEV);
	uint16_t busVoltageHigh = 1000*(MOTOR_BUS_VOLTAGE + MOTOR_BUS_VOLTAGE_DEV);
	uint16_t busVoltage = f_ina219_GetBusVoltageInMilis();

	// is there bus voltage?
	if((busVoltage >= busVoltageLow) && (busVoltage <= busVoltageHigh)) isBusOk = true;

	// power draw is within the limit? Units: miliWatts
	int16_t powerUpperLimit;
	int16_t powerLowerLimit;

	if(isMotorWorking)
	{
		uint32_t motorVoltage = busVoltage * motorVelocityOCR / MAX_MOTOR_PWM;
		uint32_t expectedPower = MOTOR_EXP_POWER_K * motorVoltage * motorVoltage / 1000;
		powerUpperLimit = expectedPower + 1000 * MOTOR_POWER_DEV - motorIdlePower;
		powerLowerLimit = expectedPower - 1000 * MOTOR_POWER_DEV - motorIdlePower;
	}
	else
	{
		powerUpperLimit = MOTOR_MAX_IDLE_POWER * 1000;
		powerLowerLimit = -MOTOR_MAX_IDLE_POWER * 1000;
	}

	uint16_t power = f_ina219_GetPowerInMilis();
	if((power >= powerLowerLimit) && (power <= powerUpperLimit)) isPowerOk = true;


	return isBusOk && isPowerOk;
}

void f_machine_MotorSetIdlePower(uint16_t power)
{
	motorIdlePower = power;
}

// ========================== sensor section =========================
/*
 * how does it work:
 * trigger Timer to work as OC mode and start counting
 * after 10us there is an  interrupt request
 * change Timer to IC mode
 * count sensorTimeElapsed until second inteerupt request
 */

static void f_work_sensorTimerModeIC()
{
	TIM_IC_InitTypeDef sConfigIC = {0};
	HAL_TIM_OC_DeInit(pSensorTIMHandler);

	if (HAL_TIM_IC_Init(pSensorTIMHandler) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(pSensorTIMHandler, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_CLEAR_IT(pSensorTIMHandler, TIM_IT_UPDATE);
}

static void f_work_sensorTimerModeOC()
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	HAL_TIM_IC_DeInit(pSensorTIMHandler);

	if (HAL_TIM_OC_Init(pSensorTIMHandler) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 10;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(pSensorTIMHandler, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_CLEAR_IT(pSensorTIMHandler, TIM_IT_UPDATE);
}

// ================= PUBLIC FUNCTIONS ================

void f_machine_SensorInitTimer(TIM_HandleTypeDef *pTIMHandler, TIM_TypeDef *pTIMaddress)
{
	pSensorTIMHandler = pTIMHandler; //assign to local handler

	pSensorTIMHandler->Instance = pTIMaddress;
	pSensorTIMHandler->Init.Prescaler = 167;
	pSensorTIMHandler->Init.CounterMode = TIM_COUNTERMODE_UP;
	pSensorTIMHandler->Init.Period = 30000;
	pSensorTIMHandler->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pSensorTIMHandler->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(pSensorTIMHandler) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_URS_ENABLE(pSensorTIMHandler); //only counter overflow will generate interrupt and not changing 'Update Generation' bit too
}

void f_machine_SensorTriggerMeasure()
{
	if(sensorStatus == SENSOR_OFF)
	{
		f_work_sensorTimerModeOC();

		HAL_GPIO_WritePin(SENSOR_TRIG_GPIO_Port, SENSOR_TRIG_Pin, GPIO_PIN_SET);
		HAL_TIM_OC_Start_IT(pSensorTIMHandler, TIM_CHANNEL_1);

		__HAL_TIM_ENABLE_IT(pSensorTIMHandler, TIM_IT_UPDATE);

		sensorStatus = SENSOR_TRIGGER;
		work_sensorMeasureDone = false;
	}
}

uint16_t f_machine_SensorGetLastMeasure() //return value in mm
{
	bool isMeasureOk = false;
	uint16_t distance;

	if(work_sensorMeasureDone)
	{
		distance = (uint32_t)((sensorTimeElapsed + SENSOR_CORRECTION) * SOUND_VELOCITY)/2000;
		if(distance <= SENSOR_MAX_DISTANCE) isMeasureOk = true;
	}

	return isMeasureOk ? distance : 0;
}

void f_machine_SensorOutputCompareCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == pSensorTIMHandler->Instance)
	{
		if(sensorStatus == SENSOR_TRIGGER)
		{
			f_work_sensorTimerModeIC();

			HAL_GPIO_WritePin(SENSOR_TRIG_GPIO_Port, SENSOR_TRIG_Pin, GPIO_PIN_RESET);
			HAL_TIM_IC_Start_IT(pSensorTIMHandler, TIM_CHANNEL_1);

			sensorStatus = SENSOR_MEASURE;
		}
	}
}

void f_machine_SensorCaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == pSensorTIMHandler->Instance)
	{
		if(sensorStatus == SENSOR_MEASURE) //rising edge
		{
			__HAL_TIM_SET_COUNTER(pSensorTIMHandler, 0);

			sensorStatus = SENSOR_READ;
		}
		else if(sensorStatus == SENSOR_READ) //falling edge
		{
			sensorTimeElapsed = __HAL_TIM_GET_COMPARE(pSensorTIMHandler, TIM_CHANNEL_1);

			work_sensorMeasureDone = true;
			HAL_TIM_IC_Stop_IT(pSensorTIMHandler, TIM_CHANNEL_1);
			sensorStatus = SENSOR_OFF;
		}

	}
}

void f_machine_SensorPeriodElapsedCallback(TIM_HandleTypeDef *htim) //timeout for distance sensor
{
	if(htim->Instance == pSensorTIMHandler->Instance)
	{
		HAL_TIM_IC_Stop_IT(pSensorTIMHandler, TIM_CHANNEL_1);
		sensorStatus = SENSOR_OFF;
		work_sensorMeasureDone = false;
	}
}



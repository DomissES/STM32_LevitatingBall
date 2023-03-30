/*
 * work.c
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#include "work.h"
#include "tim.h"
#include "ina219.h"

#define SENSOR_MAX_DISTANCE	2200
#define SENSOR_CORRECTION	5
#define SOUND_VELOCITY		343

uint16_t motorIdlePower;
uint16_t motorVelocityOCR;
uint16_t sensorTimeElapsed;
bool sensorMeasureDone;

enum {SENSOR_OFF, SENSOR_TRIGGER, SENSOR_MEASURE, SENSOR_READ} sensorStatus;

// ====================== motor section ====================

void f_work_MotorInitTimer()
{
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 4;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = MAX_MOTOR_PWM;
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

void f_work_MotorSet(bool onOff)
{
	if(onOff) HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	else HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
}

void f_work_MotorSetVelocity(uint16_t velocity)
{
	if(velocity >= MAX_MOTOR_PWM) velocity = MAX_MOTOR_PWM;
	motorVelocityOCR = velocity;

	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, velocity);
}

bool f_work_MotorTest(bool workingMotor)
{
	bool isBusOk = false;
	bool isPowerOk = false;

	uint16_t busVoltageLow = 1000*(MOTOR_BUS_VOLTAGE - MOTOR_BUS_VOLTAGE_DEV);
	uint16_t busVoltageHigh = 1000*(MOTOR_BUS_VOLTAGE + MOTOR_BUS_VOLTAGE_DEV);
	uint16_t busVoltage = f_ina219_GetBusVoltageInMilis();

	// is there bus voltage?
	if((busVoltage >= busVoltageLow) && (busVoltage <= busVoltageHigh)) isBusOk = true;

	// power draw is within the limit? Units: miliWatts
	uint16_t powerUpperLimit;
	uint16_t powerLowerLimit;

	if(workingMotor)
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

void f_work_MotorSetIdlePower(uint16_t power)
{
	motorIdlePower = power;
}

// ========================== sensor section =========================
/*
 * how does it work:
 * trigger TIM10 working as OC mode
 * after 10us interupt flag
 * change TIM10 to IC mode
 * after t_period interrupt with containing time
 */

static void f_work_sensorTimerModeIC()
{
	TIM_IC_InitTypeDef sConfigIC = {0};
	HAL_TIM_OC_DeInit(&htim10);

	if (HAL_TIM_IC_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);
}

static void f_work_sensorTimerModeOC()
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	HAL_TIM_IC_DeInit(&htim10);

	if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 10;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);
}

void f_work_SensorInitTimer()
{

	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 167;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 30000;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_URS_ENABLE(&htim10);
}

void f_work_sensorTriggerMeasure()
{
	if(sensorStatus == SENSOR_OFF)
	{
		f_work_sensorTimerModeOC();

		HAL_GPIO_WritePin(SENSOR_TRIG_GPIO_Port, SENSOR_TRIG_Pin, GPIO_PIN_SET);
		HAL_TIM_OC_Start_IT(&htim10, TIM_CHANNEL_1);

		__HAL_TIM_ENABLE_IT(&htim10, TIM_IT_UPDATE);

		sensorStatus = SENSOR_TRIGGER;
		sensorMeasureDone = false;
	}
}

uint16_t f_work_sensorGetLastMeasure() //return value in mm
{
	bool isMeasureOk = false;
	uint16_t distance;

	if(sensorMeasureDone)
	{
		distance = (uint32_t)((sensorTimeElapsed + SENSOR_CORRECTION) * SOUND_VELOCITY)/2000;
		if(distance <= SENSOR_MAX_DISTANCE) isMeasureOk = true;
	}

	return isMeasureOk ? distance : 0;

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10)
	{
		if(sensorStatus == SENSOR_TRIGGER)
		{
			f_work_sensorTimerModeIC();

			HAL_GPIO_WritePin(SENSOR_TRIG_GPIO_Port, SENSOR_TRIG_Pin, GPIO_PIN_RESET);
			HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);

			sensorStatus = SENSOR_MEASURE;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10)
	{
		if(sensorStatus == SENSOR_MEASURE) //rising edge
		{
			__HAL_TIM_SET_COUNTER(&htim10, 0);

			sensorStatus = SENSOR_READ;
		}
		else if(sensorStatus == SENSOR_READ) //falling edge
		{
			sensorTimeElapsed = __HAL_TIM_GET_COMPARE(&htim10, TIM_CHANNEL_1);

			sensorMeasureDone = true;
			HAL_TIM_IC_Stop_IT(&htim10, TIM_CHANNEL_1);
			sensorStatus = SENSOR_OFF;
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //timeout for distance sensor
{
	if(htim->Instance == TIM10)
	{
		HAL_TIM_IC_Stop_IT(&htim10, TIM_CHANNEL_1);
		sensorStatus = SENSOR_OFF;
		sensorMeasureDone = false;
	}
}

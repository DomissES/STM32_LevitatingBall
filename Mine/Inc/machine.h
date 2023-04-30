/*
 * work.h
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#ifndef INC_MACHINE_H_
#define INC_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>
#include <tim.h>

#define MAX_MOTOR_PWM			4095

#define MOTOR_BUS_VOLTAGE		15 //in Volts
#define MOTOR_BUS_VOLTAGE_DEV	2 //in Volts

// motor expected power is f(x) = kx^2
#define MOTOR_EXP_POWER_K		(0.09)
#define MOTOR_POWER_DEV			4
#define MOTOR_MAX_IDLE_POWER	3.0

extern bool work_sensorMeasureDone;

void f_machine_MotorInitTimer(TIM_HandleTypeDef *pTIMHandler, TIM_TypeDef *pTIMaddress);
void f_machine_MotorSet(bool onOff);
void f_machine_MotorSetVelocity(uint16_t velocity);
bool f_machine_MotorTestIfOk();
void f_machine_MotorSetIdlePower(uint16_t power);

void f_machine_SensorInitTimer(TIM_HandleTypeDef *pTIMHandler, TIM_TypeDef *pTIMaddress);
void f_machine_SensorTriggerMeasure();
uint16_t f_machine_SensorGetLastMeasure();

void f_machine_SensorOutputCompareCallback(TIM_HandleTypeDef *htim); //put in timer OC capture interrupt
void f_machine_SensorCaptureCallback(TIM_HandleTypeDef *htim); //put in timer IC capture interrupt
void f_machine_SensorPeriodElapsedCallback(TIM_HandleTypeDef *htim); //put in timer period elapsed interrupt


#endif /* INC_MACHINE_H_ */

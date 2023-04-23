/*
 * work.h
 *
 *  Created on: Jan 21, 2023
 *      Author: domis
 */

#ifndef INC_WORK_H_
#define INC_WORK_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_MOTOR_PWM			4095

#define MOTOR_BUS_VOLTAGE		15
#define MOTOR_BUS_VOLTAGE_DEV	2

// motor expected power is f(x) = kx2
#define MOTOR_EXP_POWER_K		(0.09)
#define MOTOR_POWER_DEV			4
#define MOTOR_MAX_IDLE_POWER	3.0

extern bool sensorMeasureDone;

void f_work_MotorInitTimer();
void f_work_MotorSet(bool onOff);
void f_work_MotorSetVelocity(uint16_t velocity);
bool f_work_MotorTest(bool workingMotor);
void f_work_MotorSetIdlePower(uint16_t power);

void f_work_SensorInitTimer();
void f_work_sensorTriggerMeasure();
uint16_t f_work_sensorGetLastMeasure();


#endif /* INC_WORK_H_ */

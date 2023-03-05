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

#define MOTOR_BUS_VOLTAGE		15
#define MOTOR_BUS_VOLTAGE_DEV	2

// motor resistance calculated as R = K1*U + K2
#define MOTOR_RESISTANCE_K1		(-0.81)
#define MOTOR_RESISTANCE_K2		(31.85)
#define MOTOR_RESISTANCE_DEV	5


void f_work_motorInitTimer();
void f_work_motorSet(bool onOff);
void f_work_motorSetVelocity(uint8_t velocity);
bool f_work_motorTest();

void f_work_sensorInitTimer();
void f_work_sensorTriggerMeasure();
uint16_t f_work_sensorGetLastMeasure();

#endif /* INC_WORK_H_ */

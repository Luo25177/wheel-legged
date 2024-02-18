//----
// @file tmotor.h
// @author mask <beloved25177@126.com>
// @brief 这里的电机操作都是对电机数组中的所有电机进行操作的，AK的控制使用的角度变量是anglerad
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
//
//----

#pragma once

#include "can.h"
#include "motorparam.h"
#include "mymath.h"

#include <stdbool.h>

typedef enum { TENTERCONTROL,
							 TEXITCONTROL,
							 TSETZERO } Tcontrol;

typedef struct {
	bool init;
	u8	 id;
	MotorValue(float) real;
	MotorValue(float) set;
	MotorMonitor monitor;
	float				 kp, kd;
	float				 lastAngleDeg;
	float				 initReadAngle;
	float				 initSetAngle;
} Tmotor;

extern Tmotor tmotor[4];

void TmotorInit(Tmotor* motor, u8 id);
void TmotorStatueControl(u8 controlword, u8 id);
void TmotorreceiveHandle(Tmotor* motor, CanRxMsg msg);
void TmotorCommunicate(Tmotor* motor, float _pos, float _speed, float _torque, float _kp, float _kd);
void TmotorRun(Tmotor* motor);
void TmotorMonitor(Tmotor* motor);
bool TmotorSeekZero(Tmotor* motor, float speed);

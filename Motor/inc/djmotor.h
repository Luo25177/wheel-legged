//----
// @file djmotor.h
// @author mask <beloved25177@126.com>
// @brief 这里的电机操作都是对电机数组中的所有电机进行操作的
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
#include "pid.h"
#include "stm32f4xx_can.h"

#include <stdlib.h>

typedef struct {
	bool init;	// 是否获得的零点位置脉冲
	u8	 id;
	s16	 n;	 // 累计圈数
	s16	 lockPulse;
	vs16 lastPulseRead;	 // 上次读到的脉冲
	vs16 pulseRead;
	vs16 temperature;
	PID* pulsePid;	// 位置环获得速度
	PID* speedPid;	// 速度环获得电流
	// float angleChange;		 // 角度变化量
	MotorValue(s16) real;	 // 读到的数据
	MotorValue(s16) set;	 // 设定值
	MotorMonitor monitor;
	vs32				 pulseAccumulate;	 // 累计脉冲
} DJmotor;

extern DJmotor djmotor[2];

void DJmotorRun(DJmotor* motor);
void DJmotorInit(DJmotor* motor, u8 id);
void DJmotorCommunicate(DJmotor* motor, u32 stdid);
void DJmotorreceiveHandle(DJmotor* motor, CanRxMsg msg);
void DJmotorMonitor(DJmotor* motor);

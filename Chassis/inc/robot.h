//----
// @file robot.h
// @author mask <beloved25177@126.com>
// @brief
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
//
//----

#pragma once

#include "btoscilloscope.h"
#include "leg.h"
#include "yesense.h"
typedef struct {
	bool flyflag;

	Leg			legL;
	Leg			legR;
	Leg			legVir;
	Yesense yesense;

	// 以下四个PID输出结果均为力
	PID yawpid;		 // 角速度控制
	PID rollpid;	 // 翻滚角控制
	PID splitpid;	 // 双腿劈叉控制

	RobotRunMode mode;

	float L0Set;	// 设定腿长，也就是当前两条腿的平均腿长
	float xSet;
	float vSet;
} Robot;

extern Robot robot;

void robotInit();
void updateState();
void balanceMode();
void jumpMode();
void haltMode();
void flyCheck();
void robotRun();

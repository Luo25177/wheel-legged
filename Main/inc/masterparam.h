//----
// @file controlparam.h
// @author mask <beloved25177@126.com>
// @brief 一些控制参数和一些控制数据
// @version 1.0
// @date 2023-11-17
//
// @copyright Copyright (c) 2023
//
//----

#pragma once

#include "robot.h"
#include "stm32f4xx.h"

#include <stdbool.h>

//----
// @brief 摇杆数据
//
//----
typedef struct {
	s16 run;		 // 跑 v cm
	s16 turn;		 // 转弯 deg
	s16 height;	 // 底盘高度 cm
	s16 tilt;		 // 底盘倾斜 deg
} HandleParam;

//----
// @brief 控制参数
//
//----
typedef struct {
	u8 begin;	 // 开始运行
	u8 stop;	 // 急停 打算做一个功能就是为了防止程序疯跑，再怎么说没有急停开关好用好像没用
	u8 robotmode;
} ControlParam;

//----
// @brief 主控参数
//
//----
typedef struct {
	HandleParam	 handle;
	ControlParam control;
} Master;

//----
// @brief 机器人状态参数，供主控查看
//
//----
typedef struct {
	u8		deviceState;	// 电机状态 0000 0000 : 0 0 WL BL FL WR BR FR
	float v;
	float height;
	float pitch;
	float yaw;
	float roll;
} RobotState;

extern Master			master;
extern RobotState robotstate;

void HandleParamInit(HandleParam* handleparam);
void ControlParamInit(ControlParam* controlparam);
void MasterInit(Master* master);
void ControlParamDeal(ControlParam* controlparam);
void HandleParamDeal(HandleParam* handleparam);
void RobotStateInit(RobotState* robotstate);
void RobotStateUpdate(RobotState* robotstate);

//----
// @file zdrive.h
// @author mask <beloved25177@126.com>
// @brief
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
//
//----

#pragma once

#include "motorparam.h"

typedef enum {
	// err,0无,1低电压,2过电压,3电流不稳,4过电流,5超速,6电阻过大,7电感过大,8编码器错误,9极对数不匹配,10KV校准失败,11模式不合法,12参数错误,13高温
	Zdrive_Well,
	Zdrive_InsufficientVoltage,
	Zdrive_OverVoltage,
	Zdrive_InstabilityCurrent,
	Zdrive_OverCurrent,
	Zdrive_OverSpeed,
	Zdrive_ExcessiveR,
	Zdrive_ExcessiveInductence,
	Zdrive_LoseEncoder1,
	Zdrive_PolesErr,
	Zdrive_VKCalibrationErr,
	Zdrive_ModeErr,				// 模式不合法
	Zdrive_ParameterErr,	// 参数错误
	Zdrive_Hot
} Zdrive_Err;

typedef struct {
	MotorValue(float) real;
	MotorValue(float) set;
	MotorMonitor monitor;
	Zdrive_Err	 err;
	float				 lockangle;
} Zdrive;

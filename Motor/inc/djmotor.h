//----
// @file djmotor.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-12
// 
// @copyright Copyright (c) 2023
// 
//----

#pragma once

#include "pid.h"
#include "mymath.h"
#include <stdlib.h>
#include "motorparam.h"
#include "stm32f4xx_can.h"

#define M3508MAXPULSE	8192
#define M3508PULSETHRESHOLD 4096
#define M3508MAXCURRENT	14745
#define M3508MAXSPEED	8550.f
#define M3508ZEROSPEED 1000.f	// 寻零或者失能的最大速度
#define M3508RATIO 19.2f
#define M3508ANGLETOPULSE	436.90667f // 角度转为编码数
#define M3508TTOI 2800.f // I = T * K 中的K，将扭矩转为电流

typedef struct {
  int n;
  u8 id;
	u8 mode;
	bool enable;
 
  s16 lockPulse;
	bool setZero;	   // 是否获得的零点位置脉冲
  
  // 超时记录
	bool timeOut;
	u32	 timeOutCnt;

	// 反馈数据
	vs32  pulseAccumulate;
	float angleRead;	// 角度 累积(deg)
	
	vs16 pulseRead;
	vs16 speedRead;
	vs16 currentRead;
	vs16 temperature;
	
  vs16 lastPulseRead;
	
	vs16 angleSet;
  vs16 output; // 最终的电流输出

	PID* pulsePid;	  // 获得速度
	PID* speedPid;	  // 获得电流
}DJmotor;

void DJmotorInit(DJmotor* motor, u8 id);
void DJreceiveHandle(DJmotor* motor, CanRxMsg msg);
void DJcompute(DJmotor* motor);

//----
// @file leg.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-12
// 
// @copyright Copyright (c) 2023
// 
//----

#pragma once

#include "tim.h"
#include "tmotor.h"
#include "djmotor.h"
#include "datastruct.h"
#include "robotparam.h"

typedef struct {
  DJmotor* wheel;
  Tmotor* front, behind;
  float angle1, angle2, angle3, angle4; // 角度计算值和读取到的真实值，是和图中的一一对应
  float angle1set, angle4set; // 角度设定值，是在初始角度之上的设定值
  datastruct angle0;
  datastruct L0;
  datastruct dis;

  char dir;

  unsigned int timer;

  // 五连杆坐标系下的坐标，原点在五连杆的中垂线上
  float xa, ya;
  float xb, yb;
  float xc, yc;
  float xd, yd;
  float xe, ye;

  float Fnow;
  float Tpnow;
  float TFnow;
  float TBnow;
  float TWheelnow;

  float Fset; // 虚拟力
  float Tpset; // 关节处扭矩
  float TFset;
  float TBset;
  float TWheelset;

  PID Fpid; // 虚拟力的pid
  float normalforce; // 地面对机器人的实际支持力
  bool flyflag;

  Input X, Xd;
  Output U;

  float K[2][6];
}Leg;

void legInit(Leg* leg, LegDir dir);
void Zjie(Leg* leg, float pitch);
void Njie(Leg* leg, float xc, float yc);
void VMC(Leg* leg);
void INVMC(Leg* leg);
void flyCheck(Leg* leg, float accely);

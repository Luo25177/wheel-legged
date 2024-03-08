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

#include "DataStruct.h"
#include "djmotor.h"
#include "robotparam.h"
#include "tim.h"
#include "tmotor.h"

typedef struct {
  int dir;

  DJmotor*   wheel;
  Tmotor*    front;
  Tmotor*    behind;
  Input      X, Xd;
  Output     U;
  DataStruct theta;
  DataStruct L0;
  DataStruct dis;
  u32        timer;
  // 五连杆坐标系下的坐标，原点在五连杆的中垂线上
  float      angle0, angle1, angle2, angle3, angle4;  // 角度计算值和读取到的真实值，是和图中的一一对应
  float      angle1set, angle4set;                    // 角度设定值，是在初始角度之上的设定值

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
  float Fset;   // 虚拟力
  float Tpset;  // 关节处扭矩
  float TFset;
  float TBset;
  float TWheelset;
  float normalforce;  // 地面对机器人的实际支持力
  float K[2][6];
  PID   L0pid;  // 虚拟力的pid 是腿长的控制
} Leg;

void LegInit(Leg* leg, int dir, DJmotor* wheel, Tmotor* front, Tmotor* behind);
void LegUpdate(Leg* leg);
void Zjie(Leg* leg, float pitch);
void Njie(Leg* leg, float xc, float yc);
void VMC(Leg* leg);
void INVMC(Leg* leg);

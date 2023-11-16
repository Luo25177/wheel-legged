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
#include "leg.h"
#include "yesense.h"
typedef struct {
  Leg legL;
  Leg legR;
  Leg legSim;
  Yesense yesense;
  // 角速度控制
  PID yawpid;
  // 翻滚角控制
  PID rollpid;
  // 双腿劈叉控制
  PID splitpid;

  u8 mode;
} Robot;

extern Robot* robot;

void robotInit();

void updateState();

void balanceMode();

void jumpMode();

void flyMode();

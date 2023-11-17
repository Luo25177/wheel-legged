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
  Leg legVir;
  Yesense yesense;
  // 以下三个PID输出结果均为力
  // 角速度控制
  PID yawpid;
  // 翻滚角控制
  PID rollpid;
  // 双腿劈叉控制
  PID splitpid;
  // 虚拟力的pid 是腿长的控制
  PID L0pid; 
  
  RobotRunMode mode;

  float L0Set; // 设定腿长，也就是当前两条腿的平均腿长
  float xSet;
  float vSet;

  bool flyflag;
} Robot;

extern Robot* robot;

void robotInit();

void updateState();

void balanceMode();

void jumpMode();

void haltMode();

void flyCheck();

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

#include "KF.h"
#include "btoscilloscope.h"
#include "leg.h"
#include "linetraj.h"
#include "vesc.h"
#include "yesense.h"

typedef struct {
  bool flyflag;
  u8 jumpPhase;

  Leg legL;
  Leg legR;
  Leg legVir;
  Yesense yesense;

  // 以下四个PID输出结果均为力
  PID yawpid;  // 角速度控制
  PID rollpid; // 翻滚角控制
  PID splitpid;// 双腿劈叉控制

  RobotRunMode mode;

  float L0Set;// 设定腿长，也就是当前两条腿的平均腿长
  float xSet;
  float vSet;
  float force;

  PID pitchpid;
  PID xpid;

  LineTraj linetraj;
} Robot;

extern Robot robot;

void RobotInit();
void UpdateState();
void BalanceMode();
void Jump();
void HaltMode();
void FlyCheck();
void RobotRun();
void WBCControl();
void RobotInvertedPendulum();

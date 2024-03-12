//----
// @file vesc.h
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

#define VESCMAXCURRENT   1
#define VESCU10POLEPAIR  21
#define VESCU8POLEPAIR   21
#define VESCU5POLEPAIR   7
#define VESCMAD1POLEPAIR 12

typedef struct {
  float duty;  // 占空比
  float kp;
  float ki;
  float kd;
  float ks;
  float kd_filter;
  float d_filter;
  float p_term;
  float i_term;
  float d_term;
  s32   err;
  s32   prv_err;
  u8    polepairs;
} VESCParam;

typedef struct {
  u8    id;
  u8    n;
  float lastangle;
  MotorValue(float) real;
  MotorValue(float) set;
  MotorMonitor monitor;
  VESCParam    param;
} VESC;

void VESCInit(VESC* vesc, u8 id);
void VESCCommunicate(VESC* vesc);
void VESCMonitor(VESC* vesc);
void VESCReceiveHandle(VESC* vesc, CanRxMsg msg);
void VESCRun(VESC* vesc);

//----
// @file zdrive.h
// @author mask <beloved25177@126.com>
// @brief 单位 r r/s
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
//
//----

#pragma once

#include "motorparam.h"

typedef struct {
  u8   id;
  u8   err;
  u8   zdriveMode;
  bool errClearFlag;
  MotorValue(float) real;
  MotorValue(float) set;
  MotorMonitor monitor;
} Zdrive;

extern Zdrive zdrive[2];

void ZdriveInit(Zdrive* motor, u8 id);
void ZdriveSetMode(Zdrive* motor, float mode);
void ZdriveRun(Zdrive* _drive);
void ZdriveMonitor(Zdrive* motor);
void ZdriveReceiveHandler(Zdrive* motor, CanRxMsg rx_msg);
void ZdriveAskState(Zdrive* motor);
void ZdriveAskErr(Zdrive* motor);
void ZdriveSetPresentPos(float angleDeg, u8 id);

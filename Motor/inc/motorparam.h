//----
// @file motorparam.h
// @author mask <beloved25177@126.com>
// @brief
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
// @details 电机需要的一些通用的参数
//----
#pragma once

#include "can.h"
#include "mymath.h"
#include "pid.h"
#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

//----
// @brief 电机模式
//
//----
typedef enum { HALT,
               POSITION,
               SPEED,
               TORQUE } MotorMode;

//----
// @brief 监测电机状态
//
//----
typedef struct {
  bool stuck;
  bool enable;
  bool timeOut;
  bool stuckRealse;
  u8   mode;
  u32  stuckCnt;
  u32  timeOutCnt;
} MotorMonitor;

//----
// @brief 电机数据
// @param current A
// @param velocity rad/s
// @param angleRad rad
// @param angleDeg °
// @param torque N·M
//----
#define MotorValueDefine(T) \
  typedef struct {          \
    volatile T     current;          \
    volatile T     velocity;         \
    volatile float angleRad;         \
    volatile float angleDeg;         \
    volatile float torque;           \
  } MotorValue_##T;
#define MotorValue(T) MotorValue_##T

MotorValueDefine(float);
MotorValueDefine(s16);

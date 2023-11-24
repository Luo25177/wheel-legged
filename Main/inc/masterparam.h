//----
// @file controlparam.h
// @author mask <beloved25177@126.com>
// @brief 一些控制参数和一些控制数据
// @version 1.0
// @date 2023-11-17
// 
// @copyright Copyright (c) 2023
// 
//----

#pragma once

#include "robot.h"
#include <stdbool.h>
#include "stm32f4xx.h"

//----
// @brief 摇杆数据
// 
//----
typedef struct {
  s16 run; // 跑
  s16 turn; // 后退
  s16 height; // 底盘高度
  s16 tilt; // 底盘倾斜
}HandleParam;

//----
// @brief 控制参数
// 
//----
typedef struct {
  bool begin; // 开始运行
  bool stop; // 急停 打算做一个功能就是为了防止程序疯跑，再怎么说没有急停开关好用
  u8 robotmode;
}ControlParam;

//----
// @brief 主控参数
// 
//----
typedef struct {
  HandleParam handle;
  ControlParam control;
}Master;

//----
// @brief 机器人状态参数，供主控查看
// 
//----
typedef struct {
  float v;
  float height;
  float pitch;
  float yaw;
  float roll;
}RobotState;

extern Master master;
extern RobotState robotstate;

void HandleParamInit(HandleParam* handleparam);
void ControlParamInit(ControlParam* controlparam);
void MasterInit(Master* master);
void ControlParamDeal(ControlParam* controlparam);
void HandleParamDeal(HandleParam* handleparam);
void RobotStateInit(RobotState* robotstate);
void RobotStateUpdate(RobotState* robotstate);

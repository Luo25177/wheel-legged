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
#include "stm32f4xx.h"

//----
// @brief 摇杆数据
// 
//----
typedef struct {
  s16 forward; // 前进
  s16 backward; // 后退
  s16 left; // 左转
  s16 right; // 右转
  s16 up; // 升高底盘
  s16 down; // 降低底盘
  s16 tiltleft; // 底盘左倾
  s16 tiltright; // 底盘右倾
}HandleParam;

//----
// @brief 控制参数
// 
//----
typedef struct {
  bool begin; // 开始运行
  bool stop; // 急停 打算做一个功能就是为了防止程序疯跑，再怎么说没有急停开关好用
}ControlParam;

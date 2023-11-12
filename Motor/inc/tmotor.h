//----
// @file tmotor.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-12
// 
// @copyright Copyright (c) 2023
// 
//----

#pragma once

#include <stdbool.h>

//电机极限参数（见电机说明）
#define P_MIN -95.5f    // 位置极限（rad）
#define P_MAX 95.5f        
#define V_MIN -50.0f    // 速度极限（Rad/s）
#define V_MAX 50.0f
#define KP_MIN 0.0f     // KP极限（N-m/rad）
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // KD极限（N-m/rad/s）
#define KD_MAX 5.0f
#define T_MIN -18.0f		//扭矩极限（Nm）
#define T_MAX 18.0f 

typedef enum {
  TPOSITION,
  TSPEED,
  TPOSSPEED
}Tmode;

typedef struct {
  float angle2Rad;
  float ratio;
  float gearradio;
}Tparam;

typedef struct {
  float angle;
  float speed;
  float torque;
  float kpBg;
  float kdBg;
  float kpSt;
  float kdSt;
  float acc;
  float accRange;
  float dcc;
  float dccRange;
  float SPStart;
  float lockRange;
}Tvalue;

typedef struct {
  unsigned int timeOutCnt;
  bool timeOut;
}Tstatus;

typedef struct {
}Tmotor;
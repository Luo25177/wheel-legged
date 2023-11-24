//----
// @file robotparam.h
// @author mask <beloved25177@126.com>
// @brief 机器人所需要的所有参数的定义，但是不完全，还需要测定
// @version 1.0
// @date 2023-11-13
// 
// @copyright Copyright (c) 2023
// 
//----
#pragma once
#include "vector.h"

#define l1 0.18f
#define l2 0.336f
#define l3 0.336f
#define l4 0.18f
#define l5 0.24f

// TODO: 重量测定和参数计算
#define MASSL1 0.1f
#define MASSL2 0.1f
#define MASSL3 0.1f
#define MASSL4 0.1f
#define MASSBODY 0.1f
#define HALFMASSBODY 0.1f
#define MASSLEG 0.1f
#define MASSWHEEL 0.1f
#define GRAVITY 9.805f
// 虚拟力前馈
#define FFEEDFORWARD -0.1f
// 最小支持力
#define FORCETHRESHOLD -20.f

#define MAXROBOTSPEED 1.f
#define MINROBOTLEGLEN 0.1f;
#define MAXROBOTLEGLEN 0.3f;

typedef enum {
  LEGLEFT = (char) -1,
  LEGRIGHT = (char) 1
}LegDir;

typedef enum {
  ROBOTNORMAL = 0,
  ROBOTJUMP,
  ROBOTHALT
}RobotRunMode;

typedef struct {
  float theta;
  float thetadot;
  float x;
  float v;
  float pitch;
  float pitchdot;
}Input;

typedef struct {
  float Tp;
  float Twheel;
}Output;

void inputInit(Input* input);
void outputInit(Output* output);

extern vector2f jumpPoints[4];
extern float Kcoeff[12][4];

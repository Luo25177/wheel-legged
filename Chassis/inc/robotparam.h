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

#include <stdbool.h>

// 初始状态：(rad)
// #define FrontAngleInit  1.762782545f
// #define BehindAngleInit 1.378810109f

#define FrontAngleInit 3.4033920414f
#define BehindAngleInit -0.2617993878f
// 杆长
#define l1 0.15f
#define l2 0.27f
#define l3 0.27f
#define l4 0.15f
#define l5 0.15f
#define WHEELR 0.1f
// 质量参数
#define MASSL1 0.2512f
#define MASSL2 0.3006f
#define MASSL3 0.33795f
#define MASSL4 0.22125f
#define MASSBODY 5.4940204f
#define HALFMASSBODY 2.7470102f
#define MASSLEG 1.111f
#define MASSWHEEL 0.71f
// 重力
#define GRAVITY 9.805f
// 虚拟力前馈
#define FFEEDFORWARD 35.f
// 最小支持力阈值，判断是否离地的支持力的阈值
#define FORCETHRESHOLD 5.f
// 机器人最大速度，如果追踪更大的速度会翻车
#define MAXROBOTSPEED 1.f
// 最小腿长
#define MINROBOTLEGLEN 0.12f;
// 最大腿长
#define MAXROBOTLEGLEN 0.40f;
// 最大腿部长度差
#define MAXROBOTLEGDIFF 0.1f;
// 最大翻滚角
#define MAXROBOTROLL 1.f;
// 最大腿部劈开的角度
#define MAXROBOTSPLIT 1;
// 腿部方向值 由于电机转动方向是相同的
#define LEGLEFT (int) -1
#define LEGRIGHT (int) 1

typedef enum { ROBOTNORMAL,
               ROBOTHALT } RobotRunMode;

typedef enum {
  JUMPBEGIN,
  JUMPKICK,
  JUMPSHRINK,
  JUMPBUFFER,
  JUMPFINISH
} JumpPhase;

typedef struct {
  float theta;
  float thetadot;
  float x;
  float v;
  float pitch;
  float pitchdot;
} Input;

typedef struct {
  float Tp;
  float Twheel;
} Output;

typedef struct {
  float s;
  float sdot;
  float psi;
  float psidot;
  float thetal;
  float thetaldot;
} WBCInput;

typedef struct {
} WBCOutput;

void InputInit(Input *input);
void OutputInit(Output *output);

extern float Kcoeff[12][4];
extern float Kcoeff_wbc[40][6];
extern float inverted_pendulum[4];

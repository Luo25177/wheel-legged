//----
// @file robotparam.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-13
// 
// @copyright Copyright (c) 2023
// @details 机器人所需要的所有参数的定义，但是不完全需要测定
//----
#pragma once

#define L1 0.18f
#define L2 0.336f
#define L3 0.336f
#define L4 0.18f
#define L5 0.24f

// TODO: 重量测定和参数计算
#define MASSL1 
#define MASSL2 
#define MASSL3 
#define MASSL4 
#define MASSWHEEL
#define MASSBODY
#define GRAVITY 9.805f

float Kcoeff[12][4] = {0};

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
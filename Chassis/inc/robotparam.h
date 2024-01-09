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

// 初始状态：(rad)
#define FrontAngleInit	3.141592653579f
#define BehindAngleInit 0.f
// 杆长
#define l1							0.18f
#define l2							0.336f
#define l3							0.336f
#define l4							0.18f
#define l5							0.24f
#define WHEELR					0.07f
// 质量参数
#define MASSL1					0.272f
#define MASSL2					0.704f
#define MASSL3					0.703f
#define MASSL4					0.272f
#define MASSBODY				5.824f
#define HALFMASSBODY		2.912f
#define MASSLEG					1.951f
#define MASSWHEEL				0.71f
// 重力
#define GRAVITY					9.805f
// 虚拟力前馈
#define FFEEDFORWARD		-28.55216f
// 最小支持力阈值，判断是否离地的支持力的阈值
#define FORCETHRESHOLD	20.f
// 机器人最大速度，如果追踪更大的速度会翻车
#define MAXROBOTSPEED		1.f
// 最小腿长
#define MINROBOTLEGLEN	0.16f;
// 最大腿长
#define MAXROBOTLEGLEN	0.4f;
// 最大腿部长度差
#define MAXROBOTLEGDIFF 0.1f;
// 最大翻滚角
#define MAXROBOTROLL		1.f;
// 最大腿部劈开的角度
#define MAXROBOTSPLIT		1;
// 腿部方向值 由于电机转动方向是相同的
#define LEGLEFT					(int) 1
#define LEGRIGHT				(int) -1

typedef enum { ROBOTNORMAL = 0,
							 ROBOTJUMP,
							 ROBOTHALT } RobotRunMode;

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

void inputInit(Input* input);
void outputInit(Output* output);

extern vector2f jumpPoints[4];
extern float		Kcoeff[12][4];

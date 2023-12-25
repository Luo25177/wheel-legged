#pragma once
//----
// @file pid.h
// @author mask (beloved25177@126.com)
// @brief
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
// @details һЩ�Ƚϳ��õ�pid
//----
#pragma once

#include "mymath.h"

typedef enum {
	PIDINC,
	PIDPOS,
	PIDTINC,
	PIDTPOS,
} pidMode;

typedef struct PID {
	float real;
	float target;
	float err[3];

	float accErr;
	float accErrLimit;
	float output;
	float outputLimit;

	float kp;
	float ki;
	float kd;

	float (*compute)(struct PID* pid, float input);
} PID;

float incCompute(PID* pid, float input);
float posCompute(PID* pid, float input);

float TincCompute(PID* pid, float input);
float TposCompute(PID* pid, float input);

float twiceIncCompute(PID* pid, float input, float inputdot);

void pidInit(PID* pid, float kp, float ki, float kd, float outputLimit, float accErrLimit, pidMode mode);

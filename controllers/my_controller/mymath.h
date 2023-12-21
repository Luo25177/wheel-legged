#pragma once

//----
// @file mymath.h
// @author mask (beloved25177@126.com)
// @brief
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
//
//----
#pragma once

#include "stdbool.h"
#include "string.h"

#define DegToRad 0.0174533f
#define RadToDeg 57.295780f
// #define sqrt(x) arm_sqrt_f32(x)
// #define abs(x) arm_abs_f32(x)
// #define abs(x) (x) > 0? (x) : -(x);
#define square(x)	 ((x) * (x));
#define max(x, y)	 (((x) > (y)) ? (x) : (y))
#define min(x, y)	 (((x) > (y)) ? (y) : (x))
#define halfPI		 1.57079633f
#define PI 3.141592653579f

#define ExternLimitInRange(T)	 extern void limitInRange_##T(T* val, T limit);
#define ExternLimitIn2Range(T) extern void limitIn2Range_##T(T* val, T min, T max);

#define limitInRange(T)	 limitInRange_##T
#define limitIn2Range(T) limitIn2Range_##T

ExternLimitInRange(int);
ExternLimitInRange(float);

ExternLimitIn2Range(int);
ExternLimitIn2Range(float);


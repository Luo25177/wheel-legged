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
#include "stm32f4xx.h"
#include "string.h"
#include "arm_math.h"

#define AngleToRad 0.0174533f
#define RadToAngle 57.295780f
#define cos(x) arm_cos_f32(x)
#define sin(x) arm_sin_f32(x)
#define sqrt(x) arm_sqrt_f32(x)
#define abs(x) arm_abs_f32(x)
#define square(x) ((x) * (x));
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) > (y)) ? (y) : (x))
#define halfPI 1.57079633f

void S16ToU8 (s16* s, u8* u); 
void U8ToS16 (u8* u, s16* s);
void F32ToU8 (float* f, u8* u);
void U8ToF32 (u8* u, float* f);

void limitInRange(float* val, float* limit);
void limitIn2Range(float* val, float* min, float* max);
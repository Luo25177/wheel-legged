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

#include "arm_math.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "string.h"

#define DegToRad  0.0174533f
#define RadToDeg  57.295780f
#define cos(x)    arm_cos_f32(x)
#define sin(x)    arm_sin_f32(x)
// #define sqrt(x) arm_sqrt_f32(x)
// #define abs(x) arm_abs_f32(x)
// #define abs(x) (x) > 0? (x) : -(x);
#define square(x) ((x) * (x));
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) > (y)) ? (y) : (x))
#define halfPI    1.57079633f

//----
// @brief 这里的数据类型转化都是小端模式，也就是低位在地址小的地方
//----
void LS16ToU8(s16* s, u8* u);
void LU8ToS16(u8* u, s16* s);

void LVS16ToU8(vs16* s, u8* u);
void LU8ToVS16(u8* u, vs16* s);

void LF32ToU8(float* f, u8* u);
void LU8ToF32(u8* u, float* f);

void LS32ToU8(int* i, u8* u);
void LU8ToS32(u8* u, int* i);

//----
// @brief 大端模式的数据转换
//----
void BS16ToU8(s16* s, u8* u);
void BU8ToS16(u8* u, s16* s);

void BVS16ToU8(vs16* s, u8* u);
void BU8ToVS16(u8* u, vs16* s);

void BF32ToU8(float* f, u8* u);
void BU8ToF32(u8* u, float* f);

void BS32ToU8(int* i, u8* u);
void BU8ToS32(int* i, u8* u);

#define ExternLimitInRange(T)  extern void LimitInRange_##T(T* val, T limit);
#define ExternLimitIn2Range(T) extern void LimitIn2Range_##T(T* val, T min, T max);

#define LimitInRange(T)  LimitInRange_##T
#define LimitIn2Range(T) LimitIn2Range_##T

ExternLimitInRange(u8);
ExternLimitInRange(s16);
ExternLimitInRange(int);
ExternLimitInRange(float);

ExternLimitIn2Range(u8);
ExternLimitIn2Range(s16);
ExternLimitIn2Range(int);
ExternLimitIn2Range(float);

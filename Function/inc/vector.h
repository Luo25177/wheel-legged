//----
// @file vector.h
// @author mask (beloved25177@126.com)
// @brief 
// @version 1.0
// @date 2023-11-11
// 
// @copyright Copyright (c) 2023
// 
//----
#pragma once

typedef struct {
  float x;
  float y;
} vector2f;

typedef struct {
  float x;
  float y;
  float z;
} vector3f;

vector2f vector2fAdd(const vector2f vec1, const vector2f vec2);
vector2f vector2fSub(const vector2f vec1, const vector2f vec2);
vector2f vector2fMulty(const vector2f vec1, const float num);
vector2f vector2fDivid(const vector2f vec1, const float num);
float vector2fDot(const vector2f vec1, const vector2f vec2);

vector3f vector3fAdd(const vector3f vec1, const vector3f vec2);
vector3f vector3fSub(const vector3f vec1, const vector3f vec2);
vector3f vector3fMulty(const vector3f vec1, const float num);
vector3f vector3fDivid(const vector3f vec1, const float num);
float vector3fDot(const vector3f vec1, const vector3f vec2);


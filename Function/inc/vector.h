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
} Vector2f;

typedef struct {
  float x;
  float y;
  float z;
} Vector3f;

Vector2f Vector2fAdd(const Vector2f vec1, const Vector2f vec2);
Vector2f Vector2fSub(const Vector2f vec1, const Vector2f vec2);
Vector2f Vector2fMulty(const Vector2f vec1, const float num);
Vector2f Vector2fDivid(const Vector2f vec1, const float num);
float    Vector2fDot(const Vector2f vec1, const Vector2f vec2);

Vector3f Vector3fAdd(const Vector3f vec1, const Vector3f vec2);
Vector3f Vector3fSub(const Vector3f vec1, const Vector3f vec2);
Vector3f Vector3fMulty(const Vector3f vec1, const float num);
Vector3f Vector3fDivid(const Vector3f vec1, const float num);
float    Vector3fDot(const Vector3f vec1, const Vector3f vec2);

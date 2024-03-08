#include "vector.h"

Vector2f Vector2fAdd(const Vector2f vec1, const Vector2f vec2) {
  Vector2f vec = {
    .x = vec1.x + vec2.x,
    .y = vec1.y + vec2.y,
  };
  return vec;
}
Vector2f Vector2fSub(const Vector2f vec1, const Vector2f vec2) {
  Vector2f vec = {
    .x = vec1.x - vec2.x,
    .y = vec1.y - vec2.y,
  };
  return vec;
}
Vector2f Vector2fMulty(const Vector2f vec1, const float num) {
  Vector2f vec = {
    .x = vec1.x * num,
    .y = vec1.y * num,
  };
  return vec;
}
Vector2f Vector2fDivid(const Vector2f vec1, const float num) {
  if (num == 0)
    return vec1;
  Vector2f vec = {
    .x = vec1.x / num,
    .y = vec1.y / num,
  };
  return vec;
}
float Vector2fDot(const Vector2f vec1, const Vector2f vec2) { return vec1.x * vec2.x + vec1.y * vec2.y; }

Vector3f Vector3fAdd(const Vector3f vec1, const Vector3f vec2) {
  Vector3f vec = {
    .x = vec1.x + vec2.x,
    .y = vec1.y + vec2.y,
    .z = vec1.z + vec2.z,
  };
  return vec;
}
Vector3f Vector3fSub(const Vector3f vec1, const Vector3f vec2) {
  Vector3f vec = {
    .x = vec1.x - vec2.x,
    .y = vec1.y - vec2.y,
    .z = vec1.z - vec2.z,
  };
  return vec;
}
Vector3f Vector3fMulty(const Vector3f vec1, const float num) {
  Vector3f vec = {
    .x = vec1.x * num,
    .y = vec1.y * num,
    .z = vec1.z * num,
  };
  return vec;
}
Vector3f Vector3fDivid(const Vector3f vec1, const float num) {
  if (num == 0)
    return vec1;
  Vector3f vec = {
    .x = vec1.x / num,
    .y = vec1.y / num,
    .z = vec1.z / num,
  };
  return vec;
}
float Vector3fDot(const Vector3f vec1, const Vector3f vec2) { return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z; }

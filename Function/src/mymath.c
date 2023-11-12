#include "mymath.h"

void LS16ToU8 (s16* s, u8* u) {
  memcpy(u, s, sizeof(s16));
}

void LU8ToS16 (u8* u, s16* s) {
  memcpy(s, u, sizeof(s16));
}
void LF32ToU8 (float* f, u8* u) {
  memcpy(u, f, sizeof(float));
}

void LU8ToF32 (u8* u, float* f) {
  memcpy(f, u, sizeof(float));
}

void BS16ToU8 (s16* s, u8* u) {
  *u = (*s >> 8) & 0xff;
  *(u + 1) = *s & 0xff;
}

void BU8ToS16 (u8* u, s16* s) {
  *s = (s16) (u[0] << 8 | u[1]);
}

void BF32ToU8 (float* f, u8* u) {
  *u = (*(int *)f >> 24) & 0xff;
  *(u + 1) = (*(int *)f >> 16) & 0xff;
  *(u + 2) = (*(int *)f >> 8) & 0xff;
  *(u + 3) = *(int *)f & 0xff;
}

void BU8ToF32 (u8* u, float* f) {
  *f = (float) ((s32) (u[0] << 24 | u[1] << 16 | u[2] << 8 | u[3]));
}

void limitInRange(float* val, float limit) {
  if(limit == 0) return;
  if(*val < -limit) *val = -limit;
  else if(*val > limit) *val = limit;
}

void limitIn2Range(float* val, float min, float max) {
  if(max == min) return;
  if(*val < min) *val = min;
  else if(*val > max) *val = max;
}
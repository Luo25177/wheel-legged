#include "mymath.h"

void S16ToU8 (s16* s, u8* u) {
  memcpy(u, s, sizeof(s16));
}

void U8ToS16 (u8* u, s16* s) {
  memcpy(s, u, sizeof(s16));
}
void F32ToU8 (float* f, u8* u) {
  memcpy(u, f, sizeof(float));
}

void U8ToF32 (u8* u, float* f) {
  memcpy(f, u, sizeof(float));
}

void limitInRange(float* val, float* limit) {
  if(*val < -*limit) *val = -*limit;
  else if(*val > *limit) *val = *limit;
}

void limitIn2Range(float* val, float* min, float* max) {
  if(*val < *min) *val = *min;
  else if(*val > *max) *val = *max;
}
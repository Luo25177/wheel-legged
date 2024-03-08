#include "mymath.h"

void LS16ToU8(s16* s, u8* u) { memcpy(u, s, sizeof(s16)); }
void LU8ToS16(u8* u, s16* s) { memcpy(s, u, sizeof(s16)); }
void LVS16ToU8(vs16* s, u8* u) { memcpy(u, (void*) s, sizeof(s16)); }
void LU8ToVS16(u8* u, vs16* s) { memcpy((void*) s, u, sizeof(s16)); }
void LF32ToU8(float* f, u8* u) { memcpy(u, f, sizeof(float)); }
void LU8ToF32(u8* u, float* f) { memcpy(f, u, sizeof(float)); }
void LS32ToU8(int* i, u8* u) { memcpy(u, i, sizeof(int)); }
void LU8ToS32(u8* u, int* i) { memcpy(i, u, sizeof(int)); }
void BS16ToU8(s16* s, u8* u) {
  *u       = (*s >> 8) & 0xff;
  *(u + 1) = *s & 0xff;
}
void BU8ToS16(u8* u, s16* s) { *s = (s16) (*(u) << 8 | *(u + 1)); }
void BVS16ToU8(vs16* s, u8* u) {
  *u       = (*s >> 8) & 0xff;
  *(u + 1) = *s & 0xff;
}
void BU8ToVS16(u8* u, vs16* s) { *s = (s16) (*u << 8 | *(u + 1)); }
void BF32ToU8(float* f, u8* u) {
  *u       = (*(int*) f >> 24) & 0xff;
  *(u + 1) = (*(int*) f >> 16) & 0xff;
  *(u + 2) = (*(int*) f >> 8) & 0xff;
  *(u + 3) = *(int*) f & 0xff;
}
void BU8ToF32(u8* u, float* f) { *f = (float) ((s32) (*u << 24 | *(u + 1) << 16 | *(u + 2) << 8 | *(u + 3))); }
void BS32ToU8(int* i, u8* u) {
  *u       = (*i >> 24) & 0xff;
  *(u + 1) = (*i >> 16) & 0xff;
  *(u + 2) = (*i >> 8) & 0xff;
  *(u + 3) = *i & 0xff;
}
void BU8ToS32(int* i, u8* u) { *i = ((s32) (*u << 24 | *(u + 1) << 16 | *(u + 2) << 8 | *(u + 3))); }
#define DefineLimitInRange(T)                    \
  void LimitInRange_##T(T* val, T limit) { \
    if (limit == 0)                        \
      return;                              \
    if (*val < -limit)                     \
      *val = -limit;                       \
    else if (*val > limit)                 \
      *val = limit;                        \
  }
#define DefineLimitIn2Range(T)                         \
  void LimitIn2Range_##T(T* val, T min, T max) { \
    if (*val < min)                              \
      *val = min;                                \
    else if (*val > max)                         \
      *val = max;                                \
  }
DefineLimitInRange(u8);
DefineLimitInRange(s16);
DefineLimitInRange(int);
DefineLimitInRange(float);

DefineLimitIn2Range(u8);
DefineLimitIn2Range(s16);
DefineLimitIn2Range(int);
DefineLimitIn2Range(float);

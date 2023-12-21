#include "mymath.h"

#define LimitInRange(T)                    \
	void limitInRange_##T(T* val, T limit) { \
		if (limit == 0)                        \
			return;                              \
		if (*val < -limit)                     \
			*val = -limit;                       \
		else if (*val > limit)                 \
			*val = limit;                        \
	}
#define LimitIn2Range(T)                         \
	void limitIn2Range_##T(T* val, T min, T max) { \
		if (*val < min)                              \
			*val = min;                                \
		else if (*val > max)                         \
			*val = max;                                \
	}
LimitInRange(int);
LimitInRange(float);

LimitIn2Range(int);
LimitIn2Range(float);


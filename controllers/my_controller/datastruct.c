#include "datastruct.h"

void datastructInit(datastruct* data, const float _now, const float _last, const float _dot, const float _ddot) {
	data->now	 = _now;
	data->last = _now;
	data->dot	 = _now;
	data->ddot = _now;
}
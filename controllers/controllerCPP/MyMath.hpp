#pragma once

#define PI 3.141592653579f

template <typename T>
void LimitInRange(T& _val, const T& _limit) {
  if (_val > _limit)
    _val = _limit;
  if (_val < -_limit)
    _val = -_limit;
}

#pragma once

#include <cmath>

enum PIDMODE {
  PIDPOSMODE,
  PIDINCMODE
};

struct PIDParam {
  float kp;
  float ki;
  float kd;
};

class PID {
  char  mode;
  float target;
  float err[3];

  float accErr;

  float pOut;
  float iOut;
  float dOut;
  float output;

  float errThreshold;
  float integralThreshold;

  float kiScale;

  PIDParam param;

  public:
  void  init(const PIDMODE& _mode, const PIDParam& _param, const float& _err_threshold, const float& _integral_threshold);
  void  setTarget(const float& _target);
  float compute(const float& _input);
  bool  errInThreshold();
};

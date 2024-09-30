#pragma once

#include <cmath>

enum pid_mode {
  pid_pos_mode,
  pid_inc_mode
};

struct pid_param {
  float kp;
  float ki;
  float kd;
};

class pid_typedef {
  char  mode;
  float target;
  float err[3];

  float err_integral;

  float p_out;
  float i_out;
  float d_out;
  float output;

  float err_threshold;
  float max_err_integral;

  float ki_scale;

  pid_param param;

  public:
  void  init(const pid_mode&, const pid_param&, const float&, const float&);
  void  set_target(const float&);
  float compute(const float&);
  bool  err_in_threshold();
};


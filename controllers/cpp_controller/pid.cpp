#include "Pid.hpp"

void pid_typedef::init(const pid_mode& _mode, const pid_param& _param, const float& _err_threshold, const float& _integral_threshold) {
  this->mode              = _mode;
  this->param             = _param;
  this->err_threshold      = _err_threshold;
  this->max_err_integral = _integral_threshold;
}

void pid_typedef::set_target(const float& _target) {
  this->target = _target;
}

float pid_typedef::compute(const float& _input) {
  this->err[0] = this->target - _input;
  if (fabsf(this->err[0]) <= this->err_threshold)
    this->err[0] = 0;
  if (fabsf(this->err[0]) <= this->max_err_integral)
    this->ki_scale = 1;
  else
    this->ki_scale = 0;
  this->err_integral += this->ki_scale * (this->err[0] + this->err[1]) * 0.5f;
  switch (this->mode) {
    case pid_pos_mode:
      this->p_out   = this->param.kp * this->err[0];
      this->i_out   = this->param.ki * this->err_integral * this->ki_scale;
      this->d_out   = this->param.kd * (this->err[0] - this->err[1]);
      this->output = this->p_out + this->i_out + this->d_out;
      break;
    case pid_inc_mode:
      this->p_out    = this->param.kp * (this->err[0] - this->err[1]);
      this->i_out    = this->param.ki * this->ki_scale * (this->err[0] + this->err[1]) * 0.5f;
      this->d_out    = this->param.kd * (this->err[0] - 2 * this->err[1] + this->err[2]);
      this->output += this->p_out + this->i_out + this->d_out;
      break;
  }
  this->err[2] = this->err[1];
  this->err[1] = this->err[0];
  return this->output;
}

bool pid_typedef::err_in_threshold() {
  if (fabsf(this->err[0]) <= this->err_threshold)
    return true;
  return false;
}
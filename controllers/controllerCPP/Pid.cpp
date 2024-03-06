#pragma once

#include "Pid.hpp"

void PID::init(const PIDMODE& _mode, const PIDParam& _param, const float& _err_threshold, const float& _integral_threshold) {
  this->mode              = _mode;
  this->param             = _param;
  this->errThreshold      = _err_threshold;
  this->integralThreshold = _integral_threshold;
}

void PID::setTarget(const float& _target) {
  this->target = _target;
}

float PID::compute(const float& _input) {
  this->err[0] = this->target - _input;
  if (fabsf(this->err[0]) <= this->errThreshold)
    this->err[0] = 0;
  if (fabsf(this->err[0]) <= this->integralThreshold)
    this->kiScale = 1;
  else
    this->kiScale = 0;
  this->accErr += this->kiScale * (this->err[0] + this->err[1]) * 0.5f;
  switch (this->mode) {
    case PIDPOSMODE:
      this->pOut   = this->param.kp * this->err[0];
      this->iOut   = this->param.ki * this->accErr * this->kiScale;
      this->dOut   = this->param.kd * (this->err[0] - this->err[1]);
      this->output = this->pOut + this->iOut + this->dOut;
      break;
    case PIDINCMODE:
      this->pOut    = this->param.kp * (this->err[0] - this->err[1]);
      this->iOut    = this->param.ki * this->kiScale * (this->err[0] + this->err[1]) * 0.5f;
      this->dOut    = this->param.kd * (this->err[0] - 2 * this->err[1] + this->err[2]);
      this->output += this->pOut + this->iOut + this->dOut;
      break;
  }
  this->err[2] = this->err[1];
  this->err[1] = this->err[0];
  return this->output;
}

bool PID::errInThreshold() {
  if (fabsf(this->err[0]) <= this->errThreshold)
    return true;
  return false;
}

#include "pid.h"

//----
// @brief 增量式pid (限幅)
// 
// @param pid 
// @param input 输入
// @return float 输出
//----
float incCompute(PID* pid, float input) {
  pid->err[0] = pid->target - input;
  pid->output = pid->kp * (pid->err[0] - pid->err[1]) + pid->ki * pid->err[0] + pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  limitInRange(&pid->output, pid->outputLimit);
  return pid->output;
}
//----
// @brief 位置式pid (限幅)
// 
// @param pid 
// @param input 输入
// @return float 输出
//----
float posCompute(PID* pid, float input) {
  pid->err[0] = pid->target - input;
  pid->accErr += pid->err[0];
  limitInRange(&pid->accErr, pid->accErrLimit);
  pid->output = pid->kp * pid->err[0] + pid->ki * pid->accErr + pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  return pid->output;
}

//----
// @brief T形积分增量式PID
// 
// @param pid 
// @param input 输入
// @return float 输出
//----
float TincCompute(PID* pid, float input) {
  pid->err[0] = pid->target - input;
  float ierr = (pid->err[0] + pid->err[1]) / 2;
  pid->output = pid->kp * (pid->err[0] - pid->err[1]) + pid->ki * ierr + pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  limitInRange(&pid->output, pid->outputLimit);
  return pid->output;
}

//----
// @brief T形积分位置式PID
// 
// @param pid 
// @param input 输入
// @return float 输出
//----
float TposCompute(PID* pid, float input) {
  pid->err[0] = pid->target - input;
  pid->accErr += (pid->err[0] + pid->err[1]) / 2;
  limitInRange(&pid->accErr, pid->accErrLimit);
  pid->output = pid->kp * pid->err[0] + pid->ki * pid->accErr + pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  return pid->output;
}

void pidInit(PID* pid, float kp, float ki, float kd, float outputLimit, float accErrLimit, pidMode mode) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->err[0] = 0;
  pid->err[1] = 0;
  pid->err[2] = 0;
  pid->outputLimit = outputLimit;
  pid->accErrLimit = accErrLimit;
}

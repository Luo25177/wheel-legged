#include "pid.h"

//----
// @brief 增量式pid (限幅)
//
// @param pid
// @param input 输入
// @return float 输出
//----
float IncCompute(PID* pid, float input) {
  pid->err[0] = pid->target - input;
  pid->output = pid->kp * (pid->err[0] - pid->err[1]) + pid->ki * pid->err[0] + pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  LimitInRange(float)(&pid->output, pid->outputLimit);
  return pid->output;
}
//----
// @brief 位置式pid (限幅)
//
// @param pid
// @param input 输入
// @return float 输出
//----
float PosCompute(PID* pid, float input) {
  pid->err[0]  = pid->target - input;
  pid->accErr += pid->err[0];
  LimitInRange(float)(&pid->accErr, pid->accErrLimit);
  pid->output = pid->kp * pid->err[0] + pid->ki * pid->accErr + pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  LimitInRange(float)(&pid->output, pid->outputLimit);
  return pid->output;
}

//----
// @brief T形积分增量式PID
//
// @param pid
// @param input 输入
// @return float 输出
//----
float TIncCompute(PID* pid, float input) {
  pid->err[0] = pid->target - input;
  float ierr  = (pid->err[0] + pid->err[1]) / 2;
  pid->output = pid->kp * (pid->err[0] - pid->err[1]) + pid->ki * ierr + pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  LimitInRange(float)(&pid->output, pid->outputLimit);
  return pid->output;
}

//----
// @brief T形积分位置式PID
//
// @param pid
// @param input 输入
// @return float 输出
//----
float TPosCompute(PID* pid, float input) {
  pid->err[0]  = pid->target - input;
  pid->accErr += (pid->err[0] + pid->err[1]) / 2;
  LimitInRange(float)(&pid->accErr, pid->accErrLimit);
  pid->output = pid->kp * pid->err[0] + pid->ki * pid->accErr + pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  LimitInRange(float)(&pid->output, pid->outputLimit);
  return pid->output;
}

//----
// @brief 二次控制，同时控制位置和速度
//
// @param pid
// @param input
// @param inputdot
// @return float
//----
float TwiceIncCompute(PID* pid, float input, float inputdot) {
  pid->err[0]  = pid->target - input;
  float errdot = 0 - inputdot;
  pid->output  = pid->kp * pid->err[0] + pid->kd * errdot;
  if (pid->ki) {
    pid->accErr += inputdot;
    LimitInRange(float)(&pid->accErr, pid->accErrLimit);
    pid->output += pid->ki * pid->accErr;
  }
  LimitInRange(float)(&pid->output, pid->outputLimit);
  return pid->output;
}

//----
// @brief PID初始化，设置kp,ki,kd 选定pid模式
//
// @param pid
// @param kp
// @param ki
// @param kd
// @param outputLimit
// @param accErrLimit
// @param mode
//----
void PidInit(PID* pid, float kp, float ki, float kd, float outputLimit, float accErrLimit, pidMode mode) {
  pid->kp          = kp;
  pid->ki          = ki;
  pid->kd          = kd;
  pid->err[0]      = 0;
  pid->err[1]      = 0;
  pid->err[2]      = 0;
  pid->outputLimit = outputLimit;
  pid->accErrLimit = accErrLimit;

  switch (mode) {
    case PIDINC:
      pid->compute = IncCompute;
      break;
    case PIDPOS:
      pid->compute = PosCompute;
      break;
    case PIDTINC:
      pid->compute = TIncCompute;
      break;
    case PIDTPOS:
      pid->compute = TPosCompute;
      break;
    default:
      pid->compute = IncCompute;
      break;
  }
}

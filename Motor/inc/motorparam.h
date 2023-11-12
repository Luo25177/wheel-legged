//----
// @file motorparam.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-12
// 
// @copyright Copyright (c) 2023
// @details 电机需要的一些通用的参数
//----

typedef enum {
  HALT,
  POSITION,
  SPEED,
  TORQUE,
  POSSPEED
}MotorMode;

typedef enum {
  FORWARD = -1,
  BACKWARD = 1
}MotorDir;
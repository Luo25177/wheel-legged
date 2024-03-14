#pragma once

#include "motorparam.h"

typedef enum {
  OK                 = 0,  // 无故障
  BAT_LOW_ERR        = 1,  // 欠压故障
  OVER_CURRENT_ERR   = 2,  // 过流
  OVER_TEMP_ERR      = 3,  // 过温
  MAGNETIC_ERR       = 4,  // 磁编码故障
  HALL_ERR_ERR       = 5,  // HALL编码故障
  NO_CALIBRATION_ERR = 6   // 未标定
} CyberGearState;          // 电机状态（故障信息）

typedef enum {
  RESET_MODE = 0,        // Reset[模式]
  CALI_MODE  = 1,        // Cali 模式[标定]
  RUN_MODE   = 2,        // Motor模式[运行]
} CyberGearContorlMode;  // 电机运行模式

typedef struct {
  uint32_t id   : 8;  // 只占8位
  uint32_t data : 16;
  uint32_t mode : 5;
  uint32_t res  : 3;
} EXTID;

typedef struct CyberGear {
  bool init;
  u8   id;
  MotorValue(float) real;
  MotorValue(float) set;
  MotorMonitor monitor;
  float        kp, kd;
  float        lastAngleDeg;
  float        initReadAngle;
  float        initSetAngle;
} CyberGear;

void CyberGearInit(CyberGear* motor, u8 id);
void CyberGearStatueControl(u8 controlword, u8 id);
void CyberGearreceiveHandle(CyberGear* motor, CanRxMsg msg);
void CyberGearCommunicate(CyberGear* motor, float _pos, float _speed, float _torque, float _kp, float _kd);
void CyberGearRun(CyberGear* motor);
void CyberGearMonitor(CyberGear* motor);

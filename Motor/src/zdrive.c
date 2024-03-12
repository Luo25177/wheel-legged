#include "zdrive.h"

#define CYBERGEAR_GEARRATIO      1.f
#define CYBERGEAR_REDUCTIONRATIO 7.75f
#define CYBERGEAR_ITOTORQUE      0.87f
#define CYBERGEAR_MAXCURRENT     20.f

// mode
typedef enum {
  Zdrive_Disable,                   // 0失能
  Zdrive_Current,                   // 1电流模式
  Zdrive_Speed,                     // 2速度模式
  Zdrive_Postion,                   // 3位置模式
  Zdrive_Test,                      // 4测试模式
  Zdrive_RVCalibration,             // 5电阻电感校准
  Zdrive_EncoderLineCalibration,    // 6编码器线性补偿
  Zdrive_EncoudeOffsetCalibration,  // 7编码器偏移校准
  Zdrive_VKCalibration,             // 8VK校准
  Zdrive_SaveSetting,               // 9保存配置
  Zdrive_EraseSetting,              // 10擦除配置
  Zdrive_ClearErr,                  // 11擦除错误
  Zdrive_Brake                      // 12刹车
} ZdriveMode;

// error
typedef enum {
  Zdrive_Well,                 // 0无
  Zdrive_InsufficientVoltage,  // 1低电压
  Zdrive_OverVoltage,          // 2过电压
  Zdrive_InstabilityCurrent,   // 3电流不稳
  Zdrive_OverCurrent,          // 4过电流
  Zdrive_OverSpeed,            // 5超速
  Zdrive_ExcessiveR,           // 6电阻过大
  Zdrive_ExcessiveInductence,  // 7电感过大
  Zdrive_LoseEncoder1,         // 8编码器错误
  Zdrive_PolesErr,             // 9极对数不匹配
  Zdrive_VKCalibrationErr,     // 10 KV校准失败
  Zdrive_ModeErr,              // 11模式不合法
  Zdrive_ParameterErr,         // 12参数错误
  Zdrive_Hot                   // 13过热
} ZdriveErr;

void ZdriveInit(Zdrive* motor, u8 id) {
  motor->id                  = id;
  motor->errClearFlag        = false;
  motor->zdriveMode          = Zdrive_Current;

  motor->set.torque          = 0;
  motor->set.velocity        = 0;
  motor->set.angleDeg        = 0;
  motor->set.angleRad        = 0;
  motor->set.current         = 0;

  motor->monitor.mode        = TORQUE;
  motor->monitor.enable      = false;
  motor->monitor.timeOut     = false;
  motor->monitor.stuckCnt    = 0;
  motor->monitor.timeOutCnt  = 0;
  motor->monitor.stuckRealse = true;
  ZdriveSetPresentPos(0, id);
  ZdriveSetMode(motor, Zdrive_Current);
  ZdriveAskState(motor);
}

void ZdriveSetMode(Zdrive* motor, float mode) {
  CanTxMsg txmsg;
  motor->zdriveMode = (u8) mode;
  txmsg.StdId       = 0x00 + motor->id;
  txmsg.IDE         = CAN_Id_Standard;
  txmsg.RTR         = CAN_RTR_Data;
  txmsg.DLC         = 0x05;
  txmsg.Data[0]     = 0x3D;
  memcpy(txmsg.Data + 1, &mode, sizeof(float));
  can2Txmsg->push(can2Txmsg, txmsg);
}

void ZdriveRun(Zdrive* motor) {
  CanTxMsg txmsg;
  txmsg.StdId = 0x00 + motor->id;
  txmsg.IDE   = CAN_Id_Standard;
  txmsg.RTR   = CAN_RTR_Data;
  if (!motor->monitor.enable) {
    motor->set.angleDeg = motor->real.angleDeg;
    goto askstate;
  }
  txmsg.DLC = 0x05;
  switch (motor->monitor.mode) {
    case POSITION:
      if (motor->zdriveMode != Zdrive_Postion)
        ZdriveSetMode(motor, Zdrive_Postion);
      txmsg.Data[0] = 0x47;
      float pos     = motor->set.angleDeg * CYBERGEAR_REDUCTIONRATIO / 360;
      memcpy(txmsg.Data + 1, &pos, sizeof(float));
      break;
    case SPEED:
      if (motor->zdriveMode != Zdrive_Speed)
        ZdriveSetMode(motor, Zdrive_Speed);
      txmsg.Data[0]  = 0x45;
      float velocity = motor->set.velocity * CYBERGEAR_REDUCTIONRATIO;
      memcpy(txmsg.Data + 1, &velocity, sizeof(float));
      break;
    case TORQUE:
      if (motor->zdriveMode != Zdrive_Current)
        ZdriveSetMode(motor, Zdrive_Current);
      txmsg.Data[0] = 0x43;
      float current = motor->set.torque / CYBERGEAR_ITOTORQUE;
      LimitInRange(float)(&current, 6);
      memcpy(txmsg.Data + 1, &current, sizeof(float));
      break;
    default:
      break;
  }
  can2Txmsg->push(can2Txmsg, txmsg);
askstate:
  ZdriveAskState(motor);
}

void ZdriveMonitor(Zdrive* motor) {
  ++(motor->monitor.timeOutCnt);
  if (motor->monitor.timeOutCnt >= 10)
    motor->monitor.timeOut = true;
}

void ZdriveReceiveHandler(Zdrive* motor, CanRxMsg msg) {
  u8 id = msg.StdId - 1;
  if (id > 8)
    return;
  (motor + id)->err                = Zdrive_Well;
  (motor + id)->monitor.timeOutCnt = 0;
  (motor + id)->monitor.timeOut    = false;
  float temp;
  switch (msg.Data[0]) {
    case 0x52:
      memcpy(&((motor + id)->real.current), msg.Data + 1, sizeof(float));
      (motor + id)->real.torque = (motor + id)->real.current * CYBERGEAR_ITOTORQUE;
      break;
    case 0x5C:
      memcpy(&(motor + id)->real.velocity, msg.Data + 1, sizeof(float));
      (motor + id)->real.velocity /= CYBERGEAR_REDUCTIONRATIO;
      (motor + id)->real.velocity *= PI * 2;
      break;
    case 0x5E:
      memcpy(&(motor + id)->real.angleDeg, msg.Data + 1, sizeof(float));
      (motor + id)->real.angleDeg /= CYBERGEAR_REDUCTIONRATIO;
      (motor + id)->real.angleDeg *= 360;
      (motor + id)->real.angleRad  = DegToRad * (motor + id)->real.angleDeg;
      break;
    case 0x3C:
      memcpy(&temp, msg.Data + 1, sizeof(float));
      (motor + id)->zdriveMode = (u8) temp;
      break;
    case 0x40:
      memcpy(&temp, msg.Data + 1, sizeof(float));
      (motor + id)->err = (u8) temp;
      break;

    default:
      break;
  }
}

void ZdriveAskState(Zdrive* motor) {
  CanTxMsg txmsg;
  txmsg.StdId   = 0x00 + motor->id;
  txmsg.IDE     = CAN_Id_Standard;
  txmsg.RTR     = CAN_RTR_Data;
  txmsg.DLC     = 0x01;
  // current
  txmsg.Data[0] = 0x52;
  can2Txmsg->push(can2Txmsg, txmsg);
  // speed
  txmsg.Data[0] = 0x5C;
  can2Txmsg->push(can2Txmsg, txmsg);
  // pos
  txmsg.Data[0] = 0x5E;
  can2Txmsg->push(can2Txmsg, txmsg);
}

void ZdriveAskErr(Zdrive* motor) {
  CanTxMsg txmsg;
  txmsg.StdId   = 0x00 + motor->id;
  txmsg.IDE     = CAN_Id_Standard;
  txmsg.RTR     = CAN_RTR_Data;
  txmsg.DLC     = 0x01;
  txmsg.Data[0] = 0x40;
  can2Txmsg->push(can2Txmsg, txmsg);
}

void ZdriveSetPresentPos(float angleDeg, u8 id) {
  CanTxMsg txmsg;
  txmsg.StdId    = 0x00 + id;
  txmsg.IDE      = CAN_Id_Standard;
  txmsg.RTR      = CAN_RTR_Data;
  txmsg.DLC      = 0x05;
  txmsg.Data[0]  = 0x5F;
  angleDeg      /= 360;
  memcpy(&txmsg.Data[1], &angleDeg, sizeof(float));
  can2Txmsg->push(can2Txmsg, txmsg);
}

#include "tmotor.h"

//----
// @brief 初始化
// 
// @param motor 
// @param id 
//----
void TmotorInit(Tmotor* motor, u8 id, MotorDir dir) {
  motor->enable = false;
  motor->id = id;
  motor->dir = dir;
  motor->initval.ratio = 7.643f;
  motor->initval.gearratio = 1.0f;
  motor->initval.spMaxpos = 60; // deg
  motor->initval.spMinpos = 10; // deg

  motor->status.stuckDet = true;
  motor->status.stuckRelease = true;

  motor->initval.maxangle = 1800;
  motor->initval.minangle = -1800;

  motor->initval.kp = 150;
  motor->initval.kd = 1.0;
  
  motor->initval.kpBg = 150;
  motor->initval.kdBg = 3;
  motor->initval.kpSt = 180;
  motor->initval.kdSt = 1.5;

  motor->initval.acc = 0;
  motor->initval.dcc = 0;
  motor->initval.accRange = 0;
  motor->initval.dccRange = 0;
  
  motor->set.spStart = 0;
  motor->set.lockRange = 1.5;
  
  motor->real.spStart = 0;
  motor->real.lockRange = 1.5;
}

//----
// @brief 电机状态控制
// 
// @param controlword 
// @param id 
//----
void TmotorStatueControl(u8 controlword, u8 id) {
  CanTxMsg txmsg;
  txmsg.StdId = 0x00 + id;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.IDE = CAN_Id_Standard;
  txmsg.DLC = 0x08;
  txmsg.Data[0] = 0xff;
  txmsg.Data[1] = 0xff;
  txmsg.Data[2] = 0xff;
  txmsg.Data[3] = 0xff;
  txmsg.Data[4] = 0xff;
  txmsg.Data[5] = 0xff;
  txmsg.Data[6] = 0xff;
  
  switch(controlword) {
    case TENTERCONTROL:
      txmsg.Data[7] = 0xfc;
      break;
    case TEXITCONTROL:
      txmsg.Data[7] = 0xfd;
      break;
    case TSETZERO:
      txmsg.Data[7] = 0xfe;	
      break;
  }
  CAN_Transmit(CAN1, &txmsg);
}

//----
// @brief 数据类型转换，主要是由于 Tmotor 这种该死的数据格式没有对齐
//----
static u16 float2uint(float x, float x_min, float x_max, u8 bits)	//浮点型化为整型
{
	float span = x_max - x_min;
	float offset = x_min;

	return (u16)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint2float(int x_int, float x_min, float x_max, int bits)	//整型化为浮点型
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

//----
// @brief 电机运动控制
// 
// @param motor 
// @param _pos 
// @param _speed 
// @param _torque 
// @param _kp 
// @param _kd 
//----
void TmotorRunControl(Tmotor* motor, float _pos, float _speed, float _torque, float _kp, float _kd) {
  _pos *= AngleToRad * motor->initval.gearratio * motor->initval.ratio;
  _speed *= AngleToRad * motor->initval.gearratio * motor->initval.ratio;
  
  u16 p = float2uint(_pos, P_MIN, P_MAX, 16);
  u16 v = float2uint(_speed, P_MIN, P_MAX, 12);
  u16 t = float2uint(_torque, P_MIN, P_MAX, 12);
  u16 kp = float2uint(_kp, P_MIN, P_MAX, 12);
  u16 kd = float2uint(_kd, P_MIN, P_MAX, 12);

  CanTxMsg txmsg;
  txmsg.StdId = 0x00 + motor->id;
  txmsg.IDE = CAN_Id_Standard;
  txmsg.RTR = CAN_RTR_Data;
  txmsg.DLC = 0x08;
  txmsg.Data[0] = (p >> 8) & 0xff;
  txmsg.Data[1] = p & 0xff;
  txmsg.Data[2] = v >> 4;
  txmsg.Data[3] = ((v & 0xf) << 4) | (kp >> 8);
  txmsg.Data[4] = kp & 0xff;
  txmsg.Data[5] = kd >> 4;
  txmsg.Data[6] = ((kd & 0xf) << 4) | (t >> 8);
  txmsg.Data[7] = t & 0xff;
  
  can1Txmsg->push(can1Txmsg, txmsg);
}

//----
// @brief 电机接收消息处理
// 
// @param motor 
// @param msg 
//----
void TmotorreceiveHandle(Tmotor* motor, CanRxMsg msg) {
	u16 p = (msg.Data[1] << 8) | msg.Data[2];		   //电机位置
	u16 v = (msg.Data[3] << 4) | (msg.Data[4] >> 4); //电机速度
	u16 t = (msg.Data[4] & 0x0f) << 8 | msg.Data[5]; //电机扭矩

  float pos = uint2float(p, P_MIN, P_MAX, 16);
  float speed = uint2float(v, P_MIN, P_MAX, 12);

  motor->real.angle = motor->dir * pos * RadToAngle / motor->initval.gearratio / motor->initval.ratio;
  motor->real.speed = motor->dir * speed * RadToAngle / motor->initval.gearratio / motor->initval.ratio;
  motor->real.torque = motor->dir * uint2float(t, T_MIN, T_MAX, 12);
}

void TmotorEnable(Tmotor* motor, u8 controlword) {
  if(controlword & 0x01) {
    motor->enable = true;
    motor->real.angleCal = motor->real.angle;
  }
  else
    motor->enable = false;
}

void TmotorRun(Tmotor* motor) {
  if(!motor->enable) return;
  switch(motor->mode) {
    case HALT:
      TmotorRunControl(motor, 0, 0, 0, 0, 0);
      break;
    case POSITION:
      TmotorRunControl(motor, motor->set.angle * motor->initval.ratio * motor->initval.gearratio, 0, motor->initval.kp, motor->initval.kd, 0);
      break;
    case SPEED:
      TmotorRunControl(motor, 0, motor->set.speed * motor->initval.ratio * motor->initval.gearratio, 0, motor->initval.kd, 0);
      break;
    case TORQUE:
      TmotorRunControl(motor, 0, 0, motor->set.torque, 0, 0);
      break;
    default:
      TmotorRunControl(motor, 0, 0, 0, 0, 0);
      break;
  }
}

//----
// @brief 设定位置
// 
// @param motor 
// @param pos rad
//----
void TmotorSetPos(Tmotor* motor, float anglerad) {
  motor->set.angle = anglerad * motor->dir;
}

void TmotorSetSpeed(Tmotor* motor, float speedrad) {
  motor->set.speed = speedrad * motor->dir;
}

void TmotorSetTorque(Tmotor* motor, float torque) {
  motor->set.torque = torque * motor->dir;
}

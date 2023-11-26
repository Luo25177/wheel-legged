#include "djmotor.h"

#define M3508MAXPULSE	8192
#define M3508PULSETHRESHOLD 4096
#define M3508MAXCURRENT	14745
#define M3508MAXSPEED	8550.f
#define M3508ZEROSPEED 1000.f	// 寻零或者失能的最大速度
#define M3508RATIO 19.2f
#define M3508ANGLETOPULSE	436.90667f // 角度转为编码数 deg->pulse
#define M3508TTOI 2800.f // I = T * K 中的K，将扭矩转为电流
#define M3508FINISHPULSETHRESHOLD 60.f // 电机到位判定的阈值

void DJmotorInit(DJmotor* motor, u8 id) {
  // 说实话这里不太敢写 while(motor); 主要是也不是很清楚单片机内部分配的空间，如果超出数组范围之后，这个motor是否会是NULL
  while(motor) {
    motor->id = id++;
    motor->n = 0;
    motor->setZero = false;
    motor->monitor.timeOut = false;
    motor->monitor.mode = TORQUE;
    motor->monitor.enable = false;
    motor->speedPid = (PID *) malloc(sizeof(PID));
    motor->pulsePid = (PID *) malloc(sizeof(PID));
    // 增量式PID
    pidInit(motor->speedPid, 8, 0.25, 0, 0, 0, PIDINC);
    pidInit(motor->pulsePid, 0.76, 0.1, 0, M3508MAXSPEED, 0, PIDINC);
    motor++;
  }
}

void DJmotorreceiveHandle(DJmotor* motor, CanRxMsg msg) {
  int id = msg.StdId - 0x201;
  BU8ToVS16(msg.Data, &motor[id].pulseRead);
  if(!motor[id].setZero) {
    motor[id].setZero = true;
    motor[id].lockPulse = motor[id].pulseRead;
  }
  if((motor[id].pulseRead - motor[id].lastPulseRead) > M3508PULSETHRESHOLD)
    motor[id].n--;
  else if((motor[id].lastPulseRead - motor[id].pulseRead) > M3508PULSETHRESHOLD)
    motor[id].n++;
  BU8ToVS16(msg.Data + 2, &motor[id].real.velocity);
  BU8ToVS16(msg.Data + 4, &motor[id].real.current);
  BU8ToVS16(msg.Data + 6, &motor[id].temperature);
  motor[id].pulseAccumulate = motor[id].n * M3508MAXPULSE + motor[id].pulseRead - motor[id].lockPulse;
  motor[id].real.angleDeg = motor[id].pulseAccumulate / M3508ANGLETOPULSE;
  motor[id].real.torque = motor[id].real.current * M3508TTOI;
}

void DJmotorCommunicate(DJmotor* motor, u8 stdid) {
  CanTxMsg txmsg;
  txmsg.IDE = CAN_Id_Standard;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.StdId = stdid;
  txmsg.DLC = 0x08;

  int index = 0;
  while(motor && index < 8) {
    BS16ToU8(&motor->set.current, &txmsg.Data[index]);
    motor++;
    index += 2;
  }
  can2Txmsg->push(can2Txmsg, txmsg);
}

void DJmotorRun(DJmotor* motor) {
  DJmotor* m = motor;
  while(motor) {
    if(!motor->monitor.enable) {
      motor->set.current = 0;
      continue;
    }
    switch (motor->monitor.mode) {
      case HALT:
        motor->set.current = 0;
        break;
      case POSITION:
        motor->pulsePid->target = motor->set.angleDeg * M3508ANGLETOPULSE;
        motor->speedPid->target = motor->pulsePid->compute(motor->pulsePid, motor->pulseAccumulate);
      case SPEED:
        motor->set.current += motor->speedPid->compute(motor->speedPid, motor->pulseAccumulate);
        limitInRange(s16) (&motor->set.current, M3508MAXCURRENT);
        break;
      case TORQUE:
        break;
      default:
        motor->set.current = 0;
        break;
    }
    motor++;
  }
  DJmotorCommunicate(m, (u8) 0x200);
}

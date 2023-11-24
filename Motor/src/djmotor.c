#include "djmotor.h"

void DJmotorInit(DJmotor* motor, u8 id) {
  // 说实话这里不太敢写 while(motor); 主要是也不是很清楚单片机内部分配的空间，如果超出数组范围之后，这个motor是否会是NULL
  while(motor) {
    motor->id = id++;
    motor->setZero = false;
    motor->timeOut = false;
    motor->mode = TORQUE;
    motor->enable = false;
    motor->n = 0;
    motor->speedPid = (PID *) malloc(sizeof(PID));
    motor->pulsePid = (PID *) malloc(sizeof(PID));
    // 增量式PID
    pidInit(motor->speedPid, 8, 0.25, 0, 0, 0, PIDINC);
    pidInit(motor->pulsePid, 0.76, 0.1, 0, M3508MAXSPEED, 0, PIDINC);
    motor++;
  }
}

void DJreceiveHandle(DJmotor* motor, CanRxMsg msg) {
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
  BU8ToVS16(msg.Data + 2, &motor[id].speedRead);
  BU8ToVS16(msg.Data + 4, &motor[id].currentRead);
  BU8ToVS16(msg.Data + 6, &motor[id].temperature);
  motor[id].pulseAccumulate = motor[id].n * M3508MAXPULSE + motor[id].pulseRead - motor[id].lockPulse;
  motor[id].angleRead = motor[id].pulseAccumulate / M3508ANGLETOPULSE;
}

void DJmotorCommunicate(DJmotor* motor, u8 stdid) {
  CanTxMsg txmsg;
  txmsg.IDE = CAN_Id_Standard;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.StdId = stdid;
  txmsg.DLC = 0x08;

  int index = 0;
  while(motor && index < 8) {
    BS16ToU8(&motor->output, &txmsg.Data[index]);
    motor++;
    index += 2;
  }
  can2Txmsg->push(can2Txmsg, txmsg);
}

void DJmotorRun(DJmotor* motor) {
  DJmotor* m = motor;
  while(motor) {
    if(!motor->enable) {
      motor->output = 0;
      continue;
    }
    switch (motor->mode) {
      case HALT:
        motor->output = 0;
        break;
      case POSITION:
        motor->pulsePid->target = motor->angleSet * M3508ANGLETOPULSE;
        motor->speedPid->target = motor->pulsePid->compute(motor->pulsePid, motor->pulseAccumulate);
      case SPEED:
        motor->output += motor->speedPid->compute(motor->speedPid, motor->pulseAccumulate);
        limitInRange(s16) (&motor->output, M3508MAXCURRENT);
        break;
      case TORQUE:
        break;
      default:
        motor->output = 0;
        break;
    }
    motor++;
  }
  DJmotorCommunicate(m, (u8) 0x200);
}

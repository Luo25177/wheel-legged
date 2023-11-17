#include "djmotor.h"

void DJmotorInit(DJmotor* motor, u8 id) {
  motor->id = id;
  motor->setZero = false;
  motor->timeOut = false;
  motor->mode = POSITION;
  motor->enable = false;
  motor->n = 0;
  motor->speedPid = (PID *) malloc(sizeof(PID));
  motor->pulsePid = (PID *) malloc(sizeof(PID));
  // 增量式PID
  pidInit(motor->speedPid, 8, 0.25, 0, 0, 0, PIDINC);
  pidInit(motor->pulsePid, 0.76, 0.1, 0, M3508MAXSPEED, 0, PIDINC);
}

void DJreceiveHandle(DJmotor* motor, CanRxMsg msg) {
  BU8ToVS16(msg.Data, &motor->pulseRead);
  if(!motor->setZero) {
    motor->setZero = true;
    motor->lockPulse = motor->pulseRead;
  }
  if((motor->pulseRead - motor->lastPulseRead) > M3508PULSETHRESHOLD)
    motor->n--;
  else if((motor->lastPulseRead - motor->pulseRead) > M3508PULSETHRESHOLD)
    motor->n++;
  BU8ToVS16(msg.Data + 2, &motor->speedRead);
  BU8ToVS16(msg.Data + 4, &motor->currentRead);
  BU8ToVS16(msg.Data + 6, &motor->temperature);
  motor->pulseAccumulate = motor->n * M3508MAXPULSE + motor->pulseRead - motor->lockPulse;
  motor->angleRead = motor->pulseAccumulate / M3508ANGLETOPULSE;
}

void DJcompute(DJmotor* motor) {
  if(!motor->enable) {
    motor->output = 0;
    return;
  }
  switch (motor->mode) {
    case HALT:
      motor->output = 0;
      break;
    case POSITION:
      motor->pulsePid->target = motor->angleSet * M3508ANGLETOPULSE;
      motor->speedPid->target = incCompute(motor->pulsePid, motor->pulseAccumulate);
    case SPEED:
      motor->output += incCompute(motor->speedPid, motor->pulseAccumulate);
      // TODO:
      // limitInRange(&motor->output, M3508MAXCURRENT);
      break;
    case TORQUE:
      break;
    default:
      motor->output = 0;
      break;
  }
}

#include "djmotor.h"

void DJmotorInit(DJmotor* motor, u8 id, MotorDir dir) {
  motor->id = id;
  motor->dir = dir;
  motor->setZero = false;
  motor->timeOut = false;
  motor->mode = POSITION;
  motor->enable = false;
  motor->n = 0;
  motor->speedPid = (PID *) malloc(sizeof(PID));
  motor->pulsePid = (PID *) malloc(sizeof(PID));
  pidInit(motor->speedPid, 8, 0.25, 0, 0, 0, INC);
  pidInit(motor->pulsePid, 0.76, 0.1, 0, M3508MAXSPEED, 0, INC);
}

void DJreceiveHandle(DJmotor* motor, CanRxMsg msg) {
  BU8ToS16(msg.Data, &motor->pulseRead);
  motor->pulseRead *= motor->dir;
  if(!motor->setZero) {
    motor->setZero = true;
    motor->lockPulse = motor->pulseRead;
  }
  if((motor->pulseRead - motor->lastPulseRead) > M3508PULSETHRESHOLD)
    motor->n--;
  else if((motor->lastPulseRead - motor->pulseRead) > M3508PULSETHRESHOLD)
    motor->n++;
  BU8ToS16(msg.Data + 2, &motor->speedRead);
  BU8ToS16(msg.Data + 4, &motor->currentRead);
  BU8ToS16(msg.Data + 6, &motor->temperature);
  motor->speedRead *= motor->dir;
  motor->currentRead *= motor->dir;
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
      motor->speedPid->target = motor->pulsePid->compute(motor->pulsePid, motor->pulseAccumulate);
    case SPEED:
      motor->output += motor->speedPid->compute(motor->speedPid, motor->pulseAccumulate);
      limitInRange(&motor->output, M3508MAXCURRENT);
      break;
    case TORQUE:
      motor->output *= motor->dir;
      break;
    default:
      motor->output = 0;
      break;
  }
}

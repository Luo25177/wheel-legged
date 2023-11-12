#include "djmotor.h"

void DJmotorInit(DJmotor* motor, u8 id) {
  motor->id = id;
  motor->isGetZero = false;
  motor->timeOut = false;
  motor->mode = DJPOSITION;
  motor->n = 0;
  motor->speedPid = (PID *) malloc(sizeof(PID));
  motor->pulsePid = (PID *) malloc(sizeof(PID));
  pidInit(motor->speedPid, 8, 0.25, 0, 0, 0, INC);
  pidInit(motor->pulsePid, 0.76, 0.1, 0, M3508MAXSPEED, 0, INC);
}

void dealCanMsg(DJmotor* motor, CanTxMsg msg) {
  BU8ToS16(msg.Data, &motor->pulseRead);
  if(!motor->isGetZero) {
    motor->isGetZero = true;
    motor->lockPulse = motor->pulseRead;
  }
  if((motor->pulseRead - motor->lastPulseRead) > M3508PULSETHRESHOLD)
    motor->n--;
  else if((motor->lastPulseRead - motor->pulseRead) > M3508PULSETHRESHOLD)
    motor->n++;
  BU8ToS16(msg.Data + 2, &motor->speedRead);
  BU8ToS16(msg.Data + 4, &motor->currentRead);
  BU8ToS16(msg.Data + 6, &motor->temperature);
  motor->pulseAccumulate = motor->n * M3508MAXPULSE + motor->pulseRead - motor->lockPulse;
  motor->angleRead = motor->pulseAccumulate / M3508ANGLETOPULSE;
}

void compute(DJmotor* motor) {
  switch (motor->mode) {
    case DJHALT:
      motor->output = 0;
      break;
    case DJPOSITION:
      motor->pulsePid->target = motor->angleSet * M3508ANGLETOPULSE;
      motor->speedPid->target = incCompute(motor->pulsePid, motor->pulseAccumulate);
    case DJSPEED:
      motor->output += incCompute(motor->speedPid, motor->speedRead);
      limitInRange(&motor->output, M3508MAXCURRENT);
      break;
    default:
      break;
  }
}

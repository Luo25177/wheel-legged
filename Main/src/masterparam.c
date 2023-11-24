#include "masterparam.h"

Master master;
RobotState robotstate;

void HandleParamInit(HandleParam* handleparam) {
  handleparam->run = 0;
  handleparam->height = 0;
  handleparam->tilt = 0;
  handleparam->turn = 0;
}

void ControlParamInit(ControlParam* controlparam) {
  controlparam->begin = false;
  controlparam->stop = true;
}

void MasterInit(Master* master) {
  HandleParamInit(&master->handle);
  ControlParamInit(&master->control);
}

void ControlParamDeal(ControlParam* controlparam) {
  robot->mode = controlparam->robotmode;
}

void HandleParamDeal(HandleParam* handleparam) {
  robot->vSet = handleparam->run;
  // TODO: 参数暂定
  robot->L0Set += handleparam->height * 0.001;
  robot->rollpid->target += handleparam->tilt * 0.001;
  robot->yawpid->target += handleparam->turn * 0.001;
}

void RobotStateInit(RobotState* robotstate) {
  robotstate->pitch = 0; //初始值应该是一个负值吧
  robotstate->yaw = 0;
  robotstate->roll = 0;
  robotstate->v = 0;
  // TODO: 参数暂定
  robotstate->height = 0.2;
}

void RobotStateUpdate(RobotState* robotstate) {
  robotstate->height = robot->legVir.L0.now;
  robotstate->pitch = robot->yesense.pitch.now;
  robotstate->yaw = robot->yesense.yaw.now;
  robotstate->roll = robot->yesense.roll.now;
  robotstate->v = robot->legVir.dis.dot;
}

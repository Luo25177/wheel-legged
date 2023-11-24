#include "masterparam.h"

Master master;

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
}

void HandleParamDeal(HandleParam* handleparam) {
  robot->vSet = handleparam->run;
  // TODO: 参数暂定
  robot->L0Set += handleparam->height * 0.001;
  robot->rollpid->target += handleparam->tilt * 0.001;
  robot->yawpid->target += handleparam->turn * 0.001;
}

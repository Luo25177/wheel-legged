#include "masterparam.h"

Master master;

void HandleParamInit(HandleParam* handleparam) {
  handleparam->forward = 0;
  handleparam->backward = 0;
  handleparam->left = 0;
  handleparam->right = 0;
  handleparam->up = 0;
  handleparam->down = 0;
  handleparam->tiltleft = 0;
  handleparam->tiltright = 0;
}

void ControlParamInit(ControlParam* controlparam) {
  controlparam->begin = false;
  controlparam->stop = true;
}

void MasterInit(Master* master) {
  HandleParamInit(&master->handle);
  ControlParamInit(&master->control);
}


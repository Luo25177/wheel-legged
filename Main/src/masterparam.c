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

void RobotStateInit(RobotState* robotstate) {
  robotstate->pitch = 0; //初始值应该是一个负值吧
  robotstate->yaw = 0;
  robotstate->roll = 0;
  robotstate->v = 0;
  // TODO: 参数暂定
  robotstate->height = 0;
}

void RobotStateUpdate(RobotState* robotstate) {

}

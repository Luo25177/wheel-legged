#include "masterparam.h"

Master     master;
RobotState robotstate;

void HandleParamInit(HandleParam* handleparam) {
  handleparam->run    = 0;
  handleparam->height = 20;
  handleparam->tilt   = 0;
  handleparam->turn   = 0;
}

void ControlParamInit(ControlParam* controlparam) {
  controlparam->begin = false;
  controlparam->stop  = true;
}

void MasterInit(Master* master) {
  HandleParamInit(&master->handle);
  ControlParamInit(&master->control);
}

void ControlParamDeal(ControlParam* controlparam) { robot.mode = controlparam->robotmode; }

void HandleParamDeal(HandleParam* handleparam) {
  // TODO: 参数暂定
  // robot.vSet						= (float) (handleparam->run * 0.01);
  // robot.L0Set					= (float) (handleparam->height * 0.01);
  // robot.rollpid.target = handleparam->tilt;
  // robot.yawpid.target	= handleparam->turn;
}

void RobotStateInit(RobotState* robotstate) {
  robotstate->pitch  = 0;  // 初始值应该是一个负值吧
  robotstate->yaw    = 0;
  robotstate->roll   = 0;
  robotstate->v      = 0;
  // TODO: 参数暂定
  robotstate->height = 0.35;
}

void RobotStateUpdate(RobotState* robotstate) {
  // TODO:
}

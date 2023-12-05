#include "masterparam.h"

Master		 master;
RobotState robotstate;

void HandleParamInit(HandleParam* handleparam) {
	handleparam->run		= 0;
	handleparam->height = 20;
	handleparam->tilt		= 0;
	handleparam->turn		= 0;
}

void ControlParamInit(ControlParam* controlparam) {
	controlparam->begin = false;
	controlparam->stop	= true;
}

void MasterInit(Master* master) {
	HandleParamInit(&master->handle);
	ControlParamInit(&master->control);
}

void ControlParamDeal(ControlParam* controlparam) { robot->mode = controlparam->robotmode; }

void HandleParamDeal(HandleParam* handleparam) {
	// TODO: 参数暂定
	robot->vSet						 = (float) (handleparam->run * 0.01);
	robot->L0Set					 = (float) (handleparam->height * 0.01);
	robot->rollpid->target = handleparam->tilt;
	robot->yawpid->target	 = handleparam->turn;
}

void RobotStateInit(RobotState* robotstate) {
	robotstate->pitch	 = 0;	 // 初始值应该是一个负值吧
	robotstate->yaw		 = 0;
	robotstate->roll	 = 0;
	robotstate->v			 = 0;
	// TODO: 参数暂定
	robotstate->height = 0.2;
}

void RobotStateUpdate(RobotState* robotstate) {
	robotstate->height = robot->legVir.L0.now;
	robotstate->pitch	 = robot->yesense.pitch.now;
	robotstate->yaw		 = robot->yesense.yaw.now;
	robotstate->roll	 = robot->yesense.roll.now;
	robotstate->v			 = robot->legVir.dis.dot;
	// 这一段写的有点傻鸟，但是没想到更好的办法
	if (tmotor[0].monitor.timeOut)
		robotstate->deviceState &= 0b11111110;
	else
		robotstate->deviceState |= 0b00000001;
	if (tmotor[1].monitor.timeOut)
		robotstate->deviceState &= 0b11111101;
	else
		robotstate->deviceState |= 0b00000010;
	if (djmotor[0].monitor.timeOut)
		robotstate->deviceState &= 0b11111011;
	else
		robotstate->deviceState |= 0b00000100;
	if (tmotor[2].monitor.timeOut)
		robotstate->deviceState &= 0b11110111;
	else
		robotstate->deviceState |= 0b00001000;
	if (tmotor[3].monitor.timeOut)
		robotstate->deviceState &= 0b11101111;
	else
		robotstate->deviceState |= 0b00010000;
	if (djmotor[1].monitor.timeOut)
		robotstate->deviceState &= 0b11011111;
	else
		robotstate->deviceState |= 0b00100000;
}

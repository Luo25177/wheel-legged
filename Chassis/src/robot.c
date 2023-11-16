#include "robot.h"

Robot* robot;

//----
// @brief 初始化
// 
//----
void robotInit() {
  robot = (Robot *) malloc(sizeof(Robot));
  legInit(&robot->legL, LEFT);
  legInit(&robot->legR, RIGHT);
  legInit(&robot->legSim, LEFT);
  // TODO: 参数暂定
  pidInit(&robot->yawpid, 1, 1, 1, 0, 0, PIDPOS);
  pidInit(&robot->rollpid, 1, 1, 1, 0, 0, PIDPOS);
  pidInit(&robot->splitpid, 1, 1, 1, 0, 0, PIDPOS);
  robot->yawpid.target = 0;
  robot->rollpid.target = 0;
  robot->splitpid.target = 0;
}

//----
// @brief 状态量更新
// 
//----
void updateState() {
  robot->yawpid.real = robot->yesense.yaw.now;
  robot->rollpid.real = robot->yesense.roll.now;
  Zjie(&robot->legL, robot->yesense.pitch.now);
  Zjie(&robot->legR, robot->yesense.pitch.now);

}

//----
// @brief lqr 控制保持平衡
// 
//----
void balanceMode() {
}

//----
// @brief 跳跃 可以先用简单的位控来实现跳跃
// 
//----
void jumpMode() {

}

//----
// @brief 腾空模式，但是感觉可以与平衡模式写到一起
// 
//----
void flyMode() {

}

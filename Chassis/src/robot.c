#include "robot.h"

Robot* robot;

//----
// @brief 初始化
// 
//----
void robotInit() {
  robot = (Robot *) malloc(sizeof(Robot));
  Tmotor* motorLF = (Tmotor *) malloc(sizeof(Tmotor));
  Tmotor* motorLB = (Tmotor *) malloc(sizeof(Tmotor));
  Tmotor* motorRF = (Tmotor *) malloc(sizeof(Tmotor));
  Tmotor* motorRB = (Tmotor *) malloc(sizeof(Tmotor));
  DJmotor* motorL = (DJmotor *) malloc(sizeof(DJmotor));
  DJmotor* motorR = (DJmotor *) malloc(sizeof(DJmotor));

  legInit(&robot->legL, LEGLEFT, motorL, motorLF, motorLB);
  legInit(&robot->legR, LEGRIGHT, motorR, motorRF, motorRB);
  legInit(&robot->legSim, LEGLEFT, NULL, NULL, NULL);
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

  robot->legSim.dis.now = (robot->legL.dis.now + robot->legR.dis.now) / 2;
  robot->legSim.L0.now = (robot->legL.L0.now + robot->legR.L0.now) / 2;
  robot->legSim.dis.dot = (robot->legL.dis.dot + robot->legR.dis.dot) / 2;
  robot->legSim.L0.dot = (robot->legL.L0.dot + robot->legR.L0.dot) / 2;

  switch(robot->mode) {
    case ROBOTNORMAL:
      balanceMode();
      break;
    case ROBOTJUMP:
      jumpMode();
      break;
    case ROBOTSOAR:
      flyMode();
      break;
    default:
      break;
  }
}

//----
// @brief lqr 控制保持平衡
// 
//----
void balanceMode() {
  float L03 = pow(robot->legSim.L0.now, 3);
  float L02 = pow(robot->legSim.L0.now, 2);
  float L01 = pow(robot->legSim.L0.now, 1);
  for(int col = 0; col < 6; col++) {
    for(int row = 0; row < 2; row++) {
      int num = col * 2 + row;
      robot->legSim.K[row][col] = Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3];
    }
  }
  robot->legSim.X.theta = robot->legSim.angle0.now;
  robot->legSim.X.thetadot = robot->legSim.angle0.dot;
  robot->legSim.X.x = robot->legSim.dis.now;
  robot->legSim.X.v = robot->legSim.dis.dot;
  robot->legSim.X.pitch = robot->yesense.pitch.now;
  robot->legSim.X.pitchdot = robot->yesense.pitch.dot;
  
  robot->legSim.Xd.theta = 0;
  robot->legSim.Xd.thetadot = 0;
  robot->legSim.Xd.x = 0;
  robot->legSim.Xd.v = 0;
  robot->legSim.Xd.pitch = 0;
  robot->legSim.Xd.pitchdot = 0;

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

void haltMode() {

}

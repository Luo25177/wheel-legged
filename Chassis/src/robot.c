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

  yesenseInit(&robot->yesense);
  legInit(&robot->legL, LEGLEFT, motorL, motorLF, motorLB);
  legInit(&robot->legR, LEGRIGHT, motorR, motorRF, motorRB);
  legInit(&robot->legVir, LEGLEFT, NULL, NULL, NULL);
  
  robot->yawpid = (PID *) malloc(sizeof(PID));
  robot->rollpid = (PID *) malloc(sizeof(PID));
  robot->splitpid = (PID *) malloc(sizeof(PID));
  robot->L0pid = (PID *) malloc(sizeof(PID));
  
  // TODO: 参数暂定 调节
  pidInit(robot->yawpid, 1, 1, 1, 0, 0, PIDPOS);
  pidInit(robot->rollpid, 1, 1, 1, 0, 0, PIDPOS);
  pidInit(robot->splitpid, 1, 1, 1, 0, 0, PIDPOS);
  pidInit(robot->L0pid, 1, 1, 1, 0, 0, PIDPOS);
  robot->yawpid->target = 0;
  robot->rollpid->target = 0;
  robot->splitpid->target = 0;
  robot->flyflag = false;
}

//----
// @brief 状态量更新和一些检查
// 
//----
void updateState() {
  Zjie(&robot->legL, robot->yesense.pitch.now);
  Zjie(&robot->legR, robot->yesense.pitch.now);
  
  robot->legL.TFnow = robot->legL.front->real.torque;
  robot->legL.TBnow = robot->legL.behind->real.torque;
  robot->legL.TWheelnow = robot->legL.wheel->currentRead * M3508CURRENTTOTORQUE;
  robot->legR.TFnow = robot->legR.front->real.torque;
  robot->legR.TBnow = robot->legR.behind->real.torque;
  robot->legR.TWheelnow = robot->legR.wheel->currentRead * M3508CURRENTTOTORQUE;
  
  flyCheck();
}

//----
// @brief lqr 控制保持平衡
// 
//----
void balanceMode() {
  robot->legVir.dis.now = (robot->legL.dis.now + robot->legR.dis.now) / 2;
  robot->legVir.L0.now = (robot->legL.L0.now + robot->legR.L0.now) / 2;
  robot->legVir.dis.dot = (robot->legL.dis.dot + robot->legR.dis.dot) / 2;
  robot->legVir.L0.dot = (robot->legL.L0.dot + robot->legR.L0.dot) / 2;
  float L03 = pow(robot->legVir.L0.now, 3);
  float L02 = pow(robot->legVir.L0.now, 2);
  float L01 = pow(robot->legVir.L0.now, 1);

  if(robot->flyflag) {
    for(int col = 0; col < 6; col++) {
      for(int row = 0; row < 2; row++) {
        int num = col * 2 + row;
        if (row == 1 && (col == 0 || col == 1))
          robot->legVir.K[row][col] = Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3];
        else 
          robot->legVir.K[row][col] = 0;
      }
    }  
  }
  else {
    for(int col = 0; col < 6; col++) {
      for(int row = 0; row < 2; row++) {
        int num = col * 2 + row;
        robot->legVir.K[row][col] = Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3];
      }
    }
  }

  robot->legVir.X.theta = robot->legVir.angle0.now;
  robot->legVir.X.thetadot = robot->legVir.angle0.dot;
  robot->legVir.X.x = robot->legVir.dis.now;
  robot->legVir.X.v = robot->legVir.dis.dot;
  robot->legVir.X.pitch = robot->yesense.pitch.now;
  robot->legVir.X.pitchdot = robot->yesense.pitch.dot;
  
  robot->legVir.Xd.theta = 0;
  robot->legVir.Xd.thetadot = 0;
  robot->legVir.Xd.x = 0;
  robot->legVir.Xd.v = 0;
  robot->legVir.Xd.pitch = 0;
  robot->legVir.Xd.pitchdot = 0;

  robot->legVir.U.Twheel = robot->legVir.K[0][0] * (robot->legVir.Xd.theta - robot->legVir.X.theta)
                         + robot->legVir.K[0][1] * (robot->legVir.Xd.thetadot - robot->legVir.X.thetadot)
                         + robot->legVir.K[0][2] * (robot->legVir.Xd.x - robot->legVir.X.x)
                         + robot->legVir.K[0][3] * (robot->legVir.Xd.v - robot->legVir.X.v)
                         + robot->legVir.K[0][4] * (robot->legVir.Xd.pitch - robot->legVir.X.pitch)
                         + robot->legVir.K[0][5] * (robot->legVir.Xd.pitchdot - robot->legVir.X.pitchdot);
  robot->legVir.U.Tp = robot->legVir.K[1][0] * (robot->legVir.Xd.theta - robot->legVir.X.theta)
                     + robot->legVir.K[1][1] * (robot->legVir.Xd.thetadot - robot->legVir.X.thetadot)
                     + robot->legVir.K[1][2] * (robot->legVir.Xd.x - robot->legVir.X.x)
                     + robot->legVir.K[1][3] * (robot->legVir.Xd.v - robot->legVir.X.v)
                     + robot->legVir.K[1][4] * (robot->legVir.Xd.pitch - robot->legVir.X.pitch)
                     + robot->legVir.K[1][5] * (robot->legVir.Xd.pitchdot - robot->legVir.X.pitchdot);
  robot->legL.TWheelset = robot->legVir.U.Twheel;
  robot->legR.TWheelset = robot->legVir.U.Twheel;
  
  robot->legL.Tpset = robot->legVir.U.Tp;
  robot->legR.Tpset = robot->legVir.U.Tp;

  robot->legL.Fset = FFEEDFORWARD;
  robot->legR.Fset = FFEEDFORWARD;
  // 补偿虚拟力
  float fCompensate = robot->L0pid->compute(robot->L0pid, robot->legVir.L0.now);
  robot->legL.Fset -= fCompensate;
  robot->legR.Fset -= fCompensate;
  // 旋转补偿
  float yawCompensate = robot->yawpid->compute(robot->yawpid, robot->yesense.yaw.now);
  robot->legL.TWheelset -= yawCompensate;
  robot->legR.TWheelset += yawCompensate;
  // 劈腿补偿
  float splitCompensate = robot->splitpid->compute(robot->splitpid, robot->legL.angle0.now - robot->legR.angle0.now);
  robot->legL.Tpset -= splitCompensate;
  robot->legR.Tpset += splitCompensate;
  // 翻滚角补偿
  float rollCompensate = robot->rollpid->compute(robot->rollpid, robot->yesense.roll.now);
  robot->legL.Fset -= splitCompensate;
  robot->legR.Fset += splitCompensate;
  
  VMC(&robot->legL);
  VMC(&robot->legR);

  // 方向值
  robot->legL.Fset *= robot->legL.dir;
  robot->legL.TFset *= robot->legL.dir;
  robot->legL.TBset *= robot->legL.dir;
  
  robot->legR.Fset *= robot->legR.dir;
  robot->legR.TFset *= robot->legR.dir;
  robot->legR.TBset *= robot->legR.dir;
}

//----
// @brief 跳跃 可以先用简单的位控来实现跳跃
// 
//----
void jumpMode() {

}

//----
// @brief 宕机模式
// 
//----
void haltMode() {

}

//----
// @brief 腾空检测
// 
//----
void flyCheck() {
  INVMC(&robot->legL);
  INVMC(&robot->legR);
  float rzwdd = robot->yesense.accelz - robot->legR.L0.ddot * cos(robot->legR.angle0.now) + 2.0 * robot->legR.angle0.dot * robot->legR.L0.dot * sin(robot->legR.angle0.now) + robot->legR.L0.now * robot->legR.angle0.ddot * sin(robot->legR.angle0.now) + robot->legR.L0.now * robot->legR.angle0.dot * robot->legR.angle0.dot * cos(robot->legR.angle0.now);
  float lzwdd = robot->yesense.accelz - robot->legL.L0.ddot * cos(robot->legL.angle0.now) + 2.0 * robot->legL.angle0.dot * robot->legL.L0.dot * sin(robot->legL.angle0.now) + robot->legL.L0.now * robot->legL.angle0.ddot * sin(robot->legL.angle0.now) + robot->legL.L0.now * robot->legL.angle0.dot * robot->legL.angle0.dot * cos(robot->legL.angle0.now);

  robot->legR.normalforce = MASSWHEEL * rzwdd + GRAVITY * MASSWHEEL + robot->legR.Fnow * cos(robot->legR.angle0.now) + robot->legR.TWheelnow * sin(robot->legR.angle0.now) / robot->legR.L0.now;
  robot->legL.normalforce = MASSWHEEL * lzwdd + GRAVITY * MASSWHEEL + robot->legL.Fnow * cos(robot->legL.angle0.now) + robot->legL.TWheelnow * sin(robot->legL.angle0.now) / robot->legL.L0.now;
  float force = (robot->legL.normalforce + robot->legR.normalforce) / 2;
  
  if (force > FORCETHRESHOLD)
    robot->flyflag = true;
  else
    robot->flyflag = false;
}

//----
// @brief 开始运行
// 
//----
void robotRun() {
  switch(robot->mode) {
    case ROBOTNORMAL:
      balanceMode();
      break;
    case ROBOTJUMP:
      jumpMode();
      break;
    case ROBOTHALT:
      haltMode();
      break;
    default:
      break;
  }
}

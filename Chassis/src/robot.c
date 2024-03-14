#include "robot.h"

Robot  robot;
Tmotor tmotor[4];
Zdrive zdrive[2];
float  expectx = 0;

//----
// @brief 初始化
//
//----
void RobotInit() {
  TmotorInit(tmotor, 1);
  TmotorInit(tmotor + 1, 2);
  TmotorInit(tmotor + 2, 3);
  TmotorInit(tmotor + 3, 4);
  ZdriveInit(zdrive, 1);
  ZdriveInit(zdrive + 1, 2);
  yesenseInit(&robot.yesense);

  LegInit(&robot.legVir, LEGLEFT, NULL, NULL, NULL);
  LegInit(&robot.legR, LEGRIGHT, &zdrive[1], &tmotor[2], &tmotor[3]);
  LegInit(&robot.legL, LEGLEFT, &zdrive[0], &tmotor[0], &tmotor[1]);

  robot.flyflag = false;
  robot.mode    = ROBOTNORMAL;

  // TODO: 参数暂定 调节
  PidInit(&robot.yawpid, 3, 0, 3, 0, 1000, PIDPOS);
  PidInit(&robot.rollpid, 1, 1, 1, 0, 1000, PIDPOS);
  PidInit(&robot.splitpid, 120, 0, 1000, 0, 1000, PIDPOS);

  robot.L0Set             = 0.18;
  robot.yawpid.target     = 0;
  robot.rollpid.target    = 0;
  robot.splitpid.target   = 0;
  robot.legL.L0pid.target = robot.L0Set;
  robot.legR.L0pid.target = robot.L0Set;
}

//----
// @brief 状态量更新和一些检查
//
//----
void UpdateState() {
  LegUpdate(&robot.legL);
  LegUpdate(&robot.legR);
  Zjie(&robot.legL, robot.yesense.pitch.now);
  Zjie(&robot.legR, robot.yesense.pitch.now);

  robot.legVir.dis.now = (-robot.legL.dis.now + robot.legR.dis.now) / 2;
  robot.legVir.dis.dot = (-robot.legL.dis.dot + robot.legR.dis.dot) / 2;
  robot.legVir.angle1  = (robot.legL.angle1 + robot.legR.angle1) / 2;
  robot.legVir.angle4  = (robot.legL.angle4 + robot.legR.angle4) / 2;

  Zjie(&robot.legVir, robot.yesense.pitch.now);

  FlyCheck();
}

//----
// @brief lqr 控制保持平衡
//
//----
void BalanceMode() {
  float L01 = robot.legVir.L0.now;
  float L02 = L01 * L01;
  float L03 = L02 * L01;

  if (robot.flyflag) {
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        int num = (row * 6) + col;
        if (row == 1 && (col == 0 || col == 1))
          robot.legVir.K[row][col] = Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3];
        else
          robot.legVir.K[row][col] = 0;
      }
    }
  } else {
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        int num                  = (row * 6) + col;
        robot.legVir.K[row][col] = Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3];
      }
    }
  }
  robot.legVir.X.theta     = robot.legVir.theta.now;
  robot.legVir.X.thetadot  = robot.legVir.theta.dot;
  robot.legVir.X.x         = robot.legVir.dis.now;
  robot.legVir.X.v         = robot.legVir.dis.dot;
  robot.legVir.X.pitch     = robot.yesense.pitch.now;
  robot.legVir.X.pitchdot  = robot.yesense.pitch.dot;

  robot.legVir.Xd.theta    = 0;
  robot.legVir.Xd.thetadot = 0;
  robot.legVir.Xd.x        = expectx;
  robot.legVir.Xd.v        = 0;
  robot.legVir.Xd.pitch    = 0;
  robot.legVir.Xd.pitchdot = 0;

  robot.legVir.U.Twheel    = robot.legVir.K[0][0] * (robot.legVir.Xd.theta - robot.legVir.X.theta) +
                          robot.legVir.K[0][1] * (robot.legVir.Xd.thetadot - robot.legVir.X.thetadot) +
                          robot.legVir.K[0][2] * (robot.legVir.Xd.x - robot.legVir.X.x) +
                          robot.legVir.K[0][3] * (robot.legVir.Xd.v - robot.legVir.X.v) +
                          robot.legVir.K[0][4] * (robot.legVir.Xd.pitch - robot.legVir.X.pitch) +
                          robot.legVir.K[0][5] * (robot.legVir.Xd.pitchdot - robot.legVir.X.pitchdot);
  robot.legVir.U.Tp = robot.legVir.K[1][0] * (robot.legVir.Xd.theta - robot.legVir.X.theta) +
                      robot.legVir.K[1][1] * (robot.legVir.Xd.thetadot - robot.legVir.X.thetadot) +
                      robot.legVir.K[1][2] * (robot.legVir.Xd.x - robot.legVir.X.x) +
                      robot.legVir.K[1][3] * (robot.legVir.Xd.v - robot.legVir.X.v) +
                      robot.legVir.K[1][4] * (robot.legVir.Xd.pitch - robot.legVir.X.pitch) +
                      robot.legVir.K[1][5] * (robot.legVir.Xd.pitchdot - robot.legVir.X.pitchdot);

  robot.legL.TWheelset   = robot.legVir.U.Twheel / 2;
  robot.legR.TWheelset   = robot.legVir.U.Twheel / 2;

  robot.legL.Tpset       = robot.legVir.U.Tp / 2;
  robot.legR.Tpset       = robot.legVir.U.Tp / 2;

  // 前馈力
  robot.legL.Fset        = FFEEDFORWARD;
  robot.legR.Fset        = FFEEDFORWARD;
  // 补偿虚拟力
  float lfCompensate     = robot.legL.L0pid.compute(&robot.legL.L0pid, robot.legL.L0.now);
  float rfCompensate     = robot.legR.L0pid.compute(&robot.legR.L0pid, robot.legR.L0.now);
  robot.legL.Fset       += lfCompensate;
  robot.legR.Fset       += rfCompensate;
  // 旋转补偿
  float yawCompensate    = robot.yawpid.compute(&robot.yawpid, robot.yesense.yaw.now);
  robot.legL.TWheelset  -= yawCompensate;
  robot.legR.TWheelset  += yawCompensate;
  // 劈腿补偿
  float splitCompensate  = robot.splitpid.compute(&robot.splitpid, robot.legL.theta.now - robot.legR.theta.now);
  robot.legL.Tpset      += splitCompensate;
  robot.legR.Tpset      -= splitCompensate;
  // 翻滚角补偿
  // float rollCompensate	 = 0;	 // robot.rollpid->compute(robot.rollpid, robot.yesense.roll.now);
  // robot.legL.Fset			-= rollCompensate;
  // robot.legR.Fset			+= rollCompensate;

  VMC(&robot.legL);
  VMC(&robot.legR);

  // 方向
  //  robot.legL.TWheelset          *= robot.legL.dir;
  robot.legL.TFset              *= robot.legL.dir;
  robot.legL.TBset              *= robot.legL.dir;

  robot.legR.TWheelset          *= robot.legR.dir;
  robot.legR.TFset              *= robot.legR.dir;
  robot.legR.TBset              *= robot.legR.dir;

  robot.legR.front->set.torque   = robot.legR.TFset;
  robot.legR.behind->set.torque  = robot.legR.TBset;
  robot.legR.wheel->set.torque   = robot.legR.TWheelset;

  robot.legL.front->set.torque   = robot.legL.TFset;
  robot.legL.behind->set.torque  = robot.legL.TBset;
  robot.legL.wheel->set.torque   = robot.legL.TWheelset;
}

//----
// @brief 跳跃
//
//----
void Jump() {
  switch (robot.jumpPhase) {
    case JUMPBEGIN:
      robot.legL.L0pid.target = MINROBOTLEGLEN;
      robot.legR.L0pid.target = MINROBOTLEGLEN;
      if (robot.legL.L0.now + robot.legR.L0.now <= 0.5)
        robot.jumpPhase++;
      break;
    case JUMPKICK:
      robot.legL.L0pid.target = MAXROBOTLEGLEN;
      robot.legR.L0pid.target = MAXROBOTLEGLEN;
      if (robot.legL.L0.now + robot.legR.L0.now >= 1.0)
        robot.jumpPhase++;
      break;
    case JUMPSHRINK:
      robot.legL.L0pid.target = MINROBOTLEGLEN;
      robot.legR.L0pid.target = MINROBOTLEGLEN;
      if (robot.force > 100)
        robot.jumpPhase++;
      break;
    case JUMPBUFFER:
      robot.legL.L0pid.target = MINROBOTLEGLEN;
      robot.legR.L0pid.target = MINROBOTLEGLEN;
      robot.jumpPhase++;
      break;
    case JUMPFINISH:
      break;
  }
}

//----
// @brief 宕机模式 实际上就是电机全部设置0电流
//
//----
void HaltMode() {
  tmotor[0].monitor.mode = HALT;
  tmotor[1].monitor.mode = HALT;
  tmotor[2].monitor.mode = HALT;
  tmotor[3].monitor.mode = HALT;
  zdrive[0].monitor.mode = HALT;
  zdrive[1].monitor.mode = HALT;
}

//----
// @brief 腾空检测
//
//----
void FlyCheck() {
  // 要计算轮子所受的地面的支持力，也就是Fn，且Fn=F+Gw 其中F应当时轮子给机体的支持力，其实就是地面给的支持力用于轮子的重力和上层机构的支持力，而对于lp是机体对轮子的力，所以要反向
  float lpw              = robot.legL.Fnow * cos(robot.legL.theta.now) - robot.legL.Tpnow * sin(robot.legL.theta.now) / robot.legL.L0.now;
  float rpw              = robot.legR.Fnow * cos(robot.legR.theta.now) - robot.legR.Tpnow * sin(robot.legR.theta.now) / robot.legR.L0.now;

  float zbdd             = (robot.yesense.accelz * cos(robot.yesense.pitch.now) - robot.yesense.accelx * sin(robot.yesense.pitch.now)) * cos(robot.yesense.roll.now) + robot.yesense.accely * sin(robot.yesense.roll.now);

  float lzwdd            = zbdd - robot.legL.L0.ddot * cos(robot.legL.theta.now) + 2 * robot.legL.L0.dot + robot.legL.theta.dot * sin(robot.legL.theta.now) + robot.legL.L0.now * robot.legL.theta.ddot * sin(robot.legL.theta.now) + robot.legL.L0.now * robot.legL.theta.dot * robot.legL.theta.dot * cos(robot.legL.theta.now);
  float rzwdd            = zbdd - robot.legR.L0.ddot * cos(robot.legR.theta.now) + 2 * robot.legR.L0.dot + robot.legR.theta.dot * sin(robot.legR.theta.now) + robot.legR.L0.now * robot.legR.theta.ddot * sin(robot.legR.theta.now) + robot.legR.L0.now * robot.legR.theta.dot * robot.legR.theta.dot * cos(robot.legR.theta.now);

  robot.legR.normalforce = rpw + MASSWHEEL * (GRAVITY + rzwdd);
  robot.legL.normalforce = lpw + MASSWHEEL * (GRAVITY + lzwdd);

  float force            = (robot.legL.normalforce + robot.legR.normalforce) / 2;

  if (force < FORCETHRESHOLD)
    robot.flyflag = true;
  else
    robot.flyflag = false;
}

//----
// @brief 开始运行
//
//----
void RobotRun() {
  switch (robot.mode) {
    case ROBOTNORMAL:
      BalanceMode();
      break;
    case ROBOTHALT:
      HaltMode();
      break;
    default:
      break;
  }
  // 将会运行所有的tmotor和djmotor
  TmotorRun(tmotor);
  TmotorRun(tmotor + 1);
  TmotorRun(tmotor + 2);
  TmotorRun(tmotor + 3);
  ZdriveRun(zdrive);
  ZdriveRun(zdrive + 1);
}

#include "robot.h"

Robot  robot;
Tmotor tmotor[4];
Zdrive zdrive[2];
VESC   vesc[4];
float  expectx         = 0;
float  expectv         = 0.f;
bool   initAngleWheel  = false;
float  initAngleWheelL = 0;
float  initAngleWheelR = 0;

float poslast          = 0;
float expecta          = 0;

bool beginRun          = false;
bool beginJump         = false;

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

  robot.flyflag   = false;
  robot.mode      = ROBOTNORMAL;
  robot.jumpPhase = JUMPFINISH;

  // TODO: 参数暂定 调节
  PidInit(&robot.yawpid, 3, 0, 90, 0, 1000, PIDPOS);
  PidInit(&robot.rollpid, 1, 1, 1, 0, 1000, PIDPOS);
  PidInit(&robot.splitpid, 50, 0, 500, 0, 1000, PIDPOS);

  robot.L0Set             = 0.15;
  robot.yawpid.target     = 0;
  robot.rollpid.target    = 0;
  robot.splitpid.target   = 0;
  robot.legL.L0pid.target = robot.L0Set;
  robot.legR.L0pid.target = robot.L0Set;

  PidInit(&robot.pitchpid, 1, 0, 0, 100, 1000, PIDPOS);
  PidInit(&robot.xpid, 1, 0, 0, 100, 1000, PIDPOS);

  LineTrajInit(&robot.linetraj, 0.1, 1.5, 0, 5, 1.5, 3.3);
}

//----
// @brief 状态量更新和一些检查
//
//----
void UpdateState() {
  LegUpdate(&robot.legL);
  LegUpdate(&robot.legR);
  float pitch = robot.yesense.pitch.now;
  Zjie(&robot.legL, pitch);
  Zjie(&robot.legR, pitch);

  if (!initAngleWheel && robot.yesense.init) {
    initAngleWheelL = PI - robot.legL.angle3.now - robot.yesense.pitch.now;
    initAngleWheelR = PI - robot.legR.angle3.now - robot.yesense.pitch.now;
    initAngleWheel  = true;
  }

  robot.legVir.angle1 = (robot.legL.angle1 + robot.legR.angle1) / 2;
  robot.legVir.angle4 = (robot.legL.angle4 + robot.legR.angle4) / 2;

  Zjie(&robot.legVir, pitch);

  float compensateXL   = (PI - robot.legL.angle3.now - robot.yesense.pitch.now - initAngleWheelL) * WHEELR;
  float compensateXR   = (PI - robot.legR.angle3.now - robot.yesense.pitch.now - initAngleWheelR) * WHEELR;
  float compensateVL   = (-robot.legL.angle3.dot - robot.yesense.pitch.dot) * WHEELR;
  float compensateVR   = (-robot.legR.angle3.dot - robot.yesense.pitch.dot) * WHEELR;

  // float compensateXL   = robot.legVir.theta.now * WHEELR;
  // float compensateXR   = robot.legVir.theta.now * WHEELR;
  // float compensateVL   = robot.legVir.theta.dot * WHEELR;
  // float compensateVR   = robot.legVir.theta.dot * WHEELR;

  float speed          = (-robot.legL.dis.now + robot.legR.dis.now - poslast) / 0.005f;
  poslast              = -robot.legL.dis.now + robot.legR.dis.now;
  robot.legVir.dis.now = (-robot.legL.dis.now + robot.legR.dis.now + compensateXL + compensateXR) / 2;
  robot.legVir.dis.dot = (speed + compensateVL + compensateVR) / 2;

  if (beginJump) {
    robot.jumpPhase = JUMPBEGIN;
    beginJump       = false;
  }
  Jump();
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
  robot.legVir.Xd.pitch    = 0;
  robot.legVir.Xd.pitchdot = 0;

  if (!beginRun) {
    robot.legVir.Xd.v = 0;
    robot.legVir.Xd.x = 0;
  } else {
    LineTrajRun(&robot.linetraj, robot.legVir.X.x);
    robot.legVir.Xd.x = robot.linetraj.xset;
    robot.legVir.Xd.v = robot.linetraj.speed * 0.5;
  }
  // if (robot.legVir.Xd.v < expectv)
  //   robot.legVir.Xd.v += expecta * 0.005;
  // else

  robot.legVir.U.Twheel = robot.legVir.K[0][0] * (robot.legVir.Xd.theta - robot.legVir.X.theta) +
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
      if (robot.legL.L0.now + robot.legR.L0.now <= 0.30)
        robot.jumpPhase++;
      break;
    case JUMPKICK:
      robot.legL.L0pid.target = MAXROBOTLEGLEN;
      robot.legR.L0pid.target = MAXROBOTLEGLEN;
      if (robot.legL.L0.now + robot.legR.L0.now >= 0.74)
        robot.jumpPhase++;
      break;
    case JUMPSHRINK:
      robot.legL.L0pid.target = 0.17;
      robot.legR.L0pid.target = 0.17;
      if (robot.force > 40)
        robot.jumpPhase++;
      break;
    case JUMPBUFFER:
      robot.legL.L0pid.target = 0.15;
      robot.legR.L0pid.target = 0.15;
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

  robot.force            = (robot.legL.normalforce + robot.legR.normalforce) / 2;

  if (robot.force < FORCETHRESHOLD)
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
      // WBCControl();
      BalanceMode();
      //      RobotInvertedPendulum();
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

void WBCControl() {
  float L_l         = robot.legL.L0.now;
  float L_r         = robot.legR.L0.now;
  float L_l2        = L_l * L_l;
  float L_r2        = L_r * L_r;
  float L_lL_r      = L_l * L_r;

  float s           = robot.legVir.dis.now;
  float s_dot       = robot.legVir.dis.dot;
  float psi         = robot.yesense.yaw.now;
  float psi_dot     = robot.yesense.yaw.dot;
  float theta_l     = robot.legL.theta.now;
  float theta_l_dot = robot.legL.theta.dot;
  float theta_r     = robot.legR.theta.now;
  float theta_r_dot = robot.legR.theta.dot;
  float phi         = robot.yesense.pitch.now;
  float phi_dot     = robot.yesense.pitch.dot;

  float K[4][10];

  if (robot.flyflag) {
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 10; col++) {
        int num = (row * 10) + col;
        if ((row != 2 && row != 3) || (col != 4 && col != 5 && col != 6 && col != 7)) {
          K[row][col] = 0;
          continue;
        }
        K[row][col] = Kcoeff_wbc[num][0] + Kcoeff_wbc[num][1] * L_l + Kcoeff_wbc[num][2] * L_r + Kcoeff_wbc[num][3] * L_l2 +
                      Kcoeff_wbc[num][4] * L_r2 + Kcoeff_wbc[num][5] * L_lL_r;
      }
    }
  } else {
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 10; col++) {
        int num     = (row * 10) + col;
        K[row][col] = Kcoeff_wbc[num][0] + Kcoeff_wbc[num][1] * L_l + Kcoeff_wbc[num][2] * L_r + Kcoeff_wbc[num][3] * L_l2 +
                      Kcoeff_wbc[num][4] * L_r2 + Kcoeff_wbc[num][5] * L_lL_r;
      }
    }
  }

  float Twl = K[0][0] * (0 - s) +
              K[0][1] * (0 - s_dot) +
              K[0][2] * (0 - psi) +
              K[0][3] * (0 - psi_dot) +
              K[0][4] * (0 - theta_l) +
              K[0][5] * (0 - theta_l_dot) +
              K[0][6] * (0 - theta_r) +
              K[0][7] * (0 - theta_r_dot) +
              K[0][8] * (0 - phi) +
              K[0][9] * (0 - phi_dot);

  float Twr = K[1][0] * (0 - s) +
              K[1][1] * (0 - s_dot) +
              K[1][2] * (0 - psi) +
              K[1][3] * (0 - psi_dot) +
              K[1][4] * (0 - theta_l) +
              K[1][5] * (0 - theta_l_dot) +
              K[1][6] * (0 - theta_r) +
              K[1][7] * (0 - theta_r_dot) +
              K[1][8] * (0 - phi) +
              K[1][9] * (0 - phi_dot);

  float Tbl = K[2][0] * (0 - s) +
              K[2][1] * (0 - s_dot) +
              K[2][2] * (0 - psi) +
              K[2][3] * (0 - psi_dot) +
              K[2][4] * (0 - theta_l) +
              K[2][5] * (0 - theta_l_dot) +
              K[2][6] * (0 - theta_r) +
              K[2][7] * (0 - theta_r_dot) +
              K[2][8] * (0 - phi) +
              K[2][9] * (0 - phi_dot);

  float Tbr = K[3][0] * (0 - s) +
              K[3][1] * (0 - s_dot) +
              K[3][2] * (0 - psi) +
              K[3][3] * (0 - psi_dot) +
              K[3][4] * (0 - theta_l) +
              K[3][5] * (0 - theta_l_dot) +
              K[3][6] * (0 - theta_r) +
              K[3][7] * (0 - theta_r_dot) +
              K[3][8] * (0 - phi) +
              K[3][9] * (0 - phi_dot);

  robot.legL.TWheelset   = Twl;
  robot.legR.TWheelset   = Twr;

  robot.legL.Tpset       = Tbl;
  robot.legR.Tpset       = Tbr;

  // 前馈力
  robot.legL.Fset        = FFEEDFORWARD;
  robot.legR.Fset        = FFEEDFORWARD;
  // 补偿虚拟力
  float lfCompensate     = robot.legL.L0pid.compute(&robot.legL.L0pid, robot.legL.L0.now);
  float rfCompensate     = robot.legR.L0pid.compute(&robot.legR.L0pid, robot.legR.L0.now);
  robot.legL.Fset       += lfCompensate;
  robot.legR.Fset       += rfCompensate;
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
  // robot.legL.TWheelset          *= robot.legL.dir;
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

void RobotInvertedPendulum() {
  float Tw = inverted_pendulum[0] * (0 - robot.legVir.dis.now) +
             inverted_pendulum[1] * (0 - robot.legVir.dis.dot) +
             inverted_pendulum[2] * (0 - robot.legVir.theta.now) +
             inverted_pendulum[3] * (0 - robot.legVir.theta.dot);

  robot.legR.wheel->set.torque = Tw;
  robot.legL.wheel->set.torque = Tw;
}

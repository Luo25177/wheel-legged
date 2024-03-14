#include "robotmonitor.h"

void LegLPidMonitor() {
  float data[6];
  data[0] = robot.legL.L0.now;
  data[1] = robot.legL.L0pid.target;
  data[2] = robot.legL.L0pid.output;
  data[3] = robot.legL.Fset;
  data[4] = robot.legL.Tpset;
  data[5] = robot.legL.theta.now;
  Oscilloscope(data, 6);
}

void LegRPidMonitor() {
  float data[6];
  data[0] = robot.legR.L0.now;
  data[1] = robot.legR.L0pid.target;
  data[2] = robot.legR.L0pid.output;
  data[3] = robot.legR.Fset;
  data[4] = robot.legR.Tpset;
  data[5] = robot.legR.theta.now;
  Oscilloscope(data, 6);
}

void RobotLqrMonitor() {
  float data[12];
  data[0] = robot.legVir.X.theta;
  data[1] = robot.legVir.X.thetadot;
  data[2] = robot.legVir.X.x;
  data[3] = robot.legVir.X.v;
  data[4] = robot.legVir.X.pitch;
  data[5] = robot.legVir.X.pitchdot;
  data[6] = robot.legL.wheel->set.torque;
  data[7] = robot.legR.wheel->set.torque;
  data[8] = robot.legL.wheel->real.torque;
  data[9] = robot.legR.wheel->real.torque;
  Oscilloscope(data, 10);
}

void RobotTorqueMonitor() {
  float data[6];
  data[0] = robot.legL.front->set.torque;
  data[1] = robot.legL.behind->set.torque;
  data[2] = robot.legL.wheel->real.torque;
  data[3] = robot.legR.front->set.torque;
  data[4] = robot.legR.behind->set.torque;
  data[5] = robot.legR.wheel->real.torque;
  Oscilloscope(data, 6);
}

void RobotWholeDataMonitor() {
  float data[12];
  data[0]  = robot.legR.theta.now;
  data[1]  = robot.legR.Fset;
  data[2]  = robot.legR.Tpset;
  data[3]  = robot.legR.TFset;
  data[4]  = robot.legR.TBset;
  data[5]  = robot.legR.TWheelset;
  data[6]  = robot.legL.theta.now;
  data[7]  = robot.legL.Fset;
  data[8]  = robot.legL.Tpset;
  data[9]  = robot.legL.TFset;
  data[10] = robot.legL.TBset;
  data[11] = robot.legL.TWheelset;
  Oscilloscope(data, 12);
}

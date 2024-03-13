#pragma once

#include "Leg.hpp"
#include "Pid.hpp"
#include "RobotParam.hpp"
#include "Sensor.hpp"

class Car : public webots::Robot {
  RobotControlMode            controlmode;
  Eigen::Matrix<float, 12, 4> SplitKCoeff;
  Eigen::Matrix<float, 6, 1>  ExpectSplit;

  Eigen::Matrix<float, 40, 6> WBCKCoeff;
  Eigen::Matrix<float, 4, 10> KWBC;
  Eigen::Matrix<float, 10, 1> StateWBC;
  Eigen::Matrix<float, 10, 1> ExpectWBC;
  Eigen::Matrix<float, 4, 1>  InputWBC;

  PID turnPID;
  PID splitPID;
  PID rollPID;

  Leg*    legL;
  Leg*    legR;
  Leg*    legVir;
  Sensor* sensor;

  JumpPhase jumpPhase;
  bool      flyflag;
  float     force;
  float     vd;

  public:
  Car();
  ~Car();
  void update();
  void flyCheck();
  void JumpControl();
  void SplitLQRControl();
  void WbcLQRControl();
  void SplitMPCControl();
};

extern PIDParam turnPIDParam;
extern PIDParam splitPIDParam;
extern PIDParam rollPIDParam;

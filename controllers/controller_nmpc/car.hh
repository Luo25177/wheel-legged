#pragma once

#include "data_struct.hh"
#include "nmpc_solver.hh"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Robot.hpp>
#include <Eigen>

using namespace webots;

#define time_step 5
#define delta_time 0.005

class Car : public Robot {
  DataStruct theta;
  DataStruct x;
  DataStruct phi;
  DataStruct tb_status;

  webots::Motor* tb_motor, *tw_motor;
  webots::PositionSensor* tb_sensor, *tw_sensor;
  webots::InertialUnit* imu;
  webots::Gyro* gyro;

  NMPCSolver nmpc;

  double tw_torque;
  double tb_torque;

public:
  Car();
  ~Car();
  
  void data_show(FILE* fp);
  void update();
  void run_nmpc();
  void run_lqr();
};

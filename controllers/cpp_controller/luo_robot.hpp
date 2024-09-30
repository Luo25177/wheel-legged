#pragma once

#include "pid.hpp"
#include "leg.hpp"
#include "sensor.hpp"
#include <webots/Robot.hpp>
#include <Eigen>

enum jump_phase_enum {
  jump_phase_halt,
  jump_phase_begin,
  jump_phase_kick,
  jump_phase_shrink,
  jump_phase_buffer,
  jump_phase_end,
};

class robot_typedef : public webots::Robot {
public:
  bool fly_flag;
  Eigen::Matrix<float, 12, 4> split_k_coef;
  Eigen::Matrix<float, 6, 1>  expect_split;

  Eigen::Matrix<float, 40, 6> wbc_k_coef;
  Eigen::Matrix<float, 4, 10> k_wbc;
  Eigen::Matrix<float, 10, 1> state_wbc;
  Eigen::Matrix<float, 10, 1> expect_wbc;
  Eigen::Matrix<float, 4, 1>  input_wbc;
  Eigen::Matrix<float, 4, 1> phi_component;

  pid_typedef split_pid;
  pid_typedef turn_pid;
  jump_phase_enum jump_phase;

  leg_typedef* leg_l;
  leg_typedef* leg_r;
  leg_typedef* leg_vir;
  sensor_typedef* sensor;

  float force;

  robot_typedef();
  void update();
  void run();
  void fly_check();
  void jump_check();
};


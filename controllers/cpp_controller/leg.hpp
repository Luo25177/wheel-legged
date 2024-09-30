#pragma once

#include "data_struct.hpp"
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "pid.hpp"
#include <Eigen>

class leg_typedef {
public:
  webots::Motor *motor1, *motor2, *motor3;
  webots::PositionSensor *sensor1, *sensor2, *sensor3;

  float angle1;
  float angle2;
  float init_angle1;
  float init_angle2;
  float l1;
  float l2;
  float angle3;

  data_struct_typedef length;
  float phi;
  float toe_x;
  float toe_y;
  data_struct_typedef theta;
  
  float f_leg_feedforward;
  float f_leg;
  float f_leg_now;
  float t_joint;
  float t_joint_now;
  data_struct_typedef dis;

  float t_motor1, t_motor2, t_motor3;
  float t_motor1_now, t_motor2_now, t_motor3_now;

  pid_typedef length_pid;

  Eigen::Matrix<float, 2, 6> split_k;
  Eigen::Matrix<float, 6, 1> state_split;
  Eigen::Matrix<float, 2, 1> input_split;

  leg_typedef(const std::string&, const std::string&, const std::string&,
    const std::string&, const std::string&, const std::string&,
    const float&, const float&, const float&);
  void zjie(const float&);
  void vmc();
  void invmc();
  void run_motor();
  void set_length(const float&);
  void update();
};


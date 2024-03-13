#pragma once

#include "RobotParam.hpp"

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

class Joint {
  protected:
  int                     dir;
  webots::Motor*          motor;
  webots::PositionSensor* encoder;
  float                   lastpos;
  float                   initAngle;
  float                   posSet;
  float                   velocitySet;
  float                   torqueSet;
  float                   torqueLimit;

  public:
  float posRead;
  float velocityRead;
  float torqueRead;
  explicit Joint(const int& _dir, const std::string& encoder_name, const std::string& motor_name, const float& _init_angle, const float& _torque_limit);
  ~Joint();
  void update();
  void setTorque(const float& _torque);
  void run();
};

class Wheel : public Joint {
  public:
  void update();
  explicit Wheel(const int& _dir, const std::string& encoder_name, const std::string& motor_name, const float& _init_angle, const float& _torque_limit)
    : Joint(_dir, encoder_name, motor_name, _init_angle, _torque_limit) { }
};

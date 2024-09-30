#pragma once

#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include "data_struct.hpp"
#include "param.hpp"

class sensor_typedef {
public:
  webots::Gyro* gyro;
  webots::InertialUnit* imu;
  webots::Accelerometer* accel;
  
  data_struct_typedef pitch;
  data_struct_typedef yaw;
  data_struct_typedef roll;
  
  float accel_x;
  float accel_y;
  float accel_z;

  sensor_typedef(const std::string&, const std::string&, const std::string&);
  void update();
};


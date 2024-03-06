#pragma once

#include "MyMath.hpp"
#include "RobotParam.hpp"

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>

class Sensor {
  webots::GPS*           gps;
  webots::InertialUnit*  imu;
  webots::Gyro*          gyro;
  webots::Accelerometer* acceler;
  webots::Camera*        camera;

  public:
  DataStruct yaw;
  DataStruct pitch;
  DataStruct roll;
  DataStruct x;
  DataStruct y;

  float accelx;
  float accely;
  float accelz;
  float z;

  Sensor();
  ~Sensor();
  void update();
};

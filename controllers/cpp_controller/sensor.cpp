#include "sensor.hpp"

sensor_typedef::sensor_typedef(const std::string& gyro_name, const std::string& imu_name, const std::string& accel_name) {
  this->gyro = new webots::Gyro(gyro_name);
  this->imu = new webots::InertialUnit(imu_name);
  this->accel = new webots::Accelerometer(accel_name);

  this->gyro->enable(time_step);
  this->imu->enable(time_step);
  this->accel->enable(time_step);

  this->pitch = { 0 };
  this->yaw = { 0 };
  this->roll = { 0 };
}

void sensor_typedef::update() {
  this->accel_x = this->accel->getValues()[0];
  this->accel_y = this->accel->getValues()[2];
  this->accel_z = this->accel->getValues()[1];

  this->pitch.dot = this->gyro->getValues()[0];
  this->yaw.dot = this->gyro->getValues()[1];
  this->pitch.dot = this->gyro->getValues()[2];

  this->pitch.now = this->imu->getRollPitchYaw()[1];
  this->yaw.now = this->imu->getRollPitchYaw()[2];
  this->roll.now = this->imu->getRollPitchYaw()[0];
}



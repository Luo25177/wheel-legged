#include "Sensor.hpp"

Sensor::Sensor() {
  this->camera  = new webots::Camera("camera");
  this->gps     = new webots::GPS("gps");
  this->imu     = new webots::InertialUnit("imu");
  this->gyro    = new webots::Gyro("gyro");
  this->acceler = new webots::Accelerometer("accel");

  this->camera->enable(TIMESTEP);
  this->gps->enable(TIMESTEP);
  this->imu->enable(TIMESTEP);
  this->gyro->enable(TIMESTEP);
  this->acceler->enable(TIMESTEP);
}

Sensor::~Sensor() {
  delete this->camera;
  delete this->gps;
  delete this->imu;
  delete this->gyro;
  delete this->acceler;
}

void Sensor::update() {
  this->pitch.now = -this->imu->getRollPitchYaw()[0];
  this->pitch.dot = -this->gyro->getValues()[0];
  this->roll.now  = this->imu->getRollPitchYaw()[1];
  this->roll.dot  = this->gyro->getValues()[2];
  float yaw_get   = this->imu->getRollPitchYaw()[2];
  if (yaw_get - this->yaw.last > 1.5 * PI)
    this->yaw.now += yaw_get - this->yaw.last - 2 * PI;
  else if (yaw_get - this->yaw.last < -1.5 * PI)
    this->yaw.now += yaw_get - this->yaw.last + 2 * PI;
  else
    this->yaw.now += yaw_get - this->yaw.last;
  this->yaw.last = this->yaw.now;

  this->yaw.dot  = this->gyro->getValues()[1];
  this->accelx   = this->acceler->getValues()[2];
  this->accely   = this->acceler->getValues()[0];
  this->accelz   = this->acceler->getValues()[1];

  this->z        = this->gps->getValues()[1];
  this->y.now    = this->gps->getValues()[0];
  this->x.now    = this->gps->getValues()[2];

  this->x.dot    = (this->x.now - this->x.last) / DT;
  this->x.last   = this->x.now;
  this->y.dot    = (this->y.now - this->y.last) / DT;
  this->y.last   = this->y.now;
}

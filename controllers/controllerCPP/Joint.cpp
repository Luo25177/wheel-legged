#include "Joint.hpp"

Joint::Joint(const int& _dir, const std::string& encoder_name, const std::string& motor_name, const float& _init_angle, const float& _torque_limit) {
  this->dir         = _dir;
  this->initAngle   = _init_angle;
  this->motor       = new webots::Motor(motor_name);
  this->encoder     = new webots::PositionSensor(encoder_name);
  this->torqueLimit = _torque_limit;
  this->encoder->enable(TIMESTEP);
  this->motor->enableTorqueFeedback(TIMESTEP);
}

void Joint::update() {
  this->torqueRead   = this->torqueSet;
  this->posRead      = this->initAngle - this->encoder->getValue() * dir;
  this->velocityRead = (this->posRead - this->lastpos) / DT;
  this->lastpos      = this->posRead;
}

void Joint::setTorque(const float& _torque) {
  this->torqueSet = _torque;
}

void Joint::run() {
  LimitInRange<float>(this->torqueSet, this->torqueLimit);
  this->motor->setTorque(this->torqueSet * this->dir);
}

Joint::~Joint() {
  delete this->motor;
  delete this->encoder;
}

void Wheel::update() {
  this->torqueRead   = this->torqueSet;
  this->posRead      = this->initAngle + this->encoder->getValue() * dir;
  this->velocityRead = (this->posRead - this->lastpos) / DT;
  this->lastpos      = this->posRead;
}


#include "leg.hpp"
#include "param.hpp"
#include <Eigen>

pid_param leg_length_pid = { 500, 0, 6000 };

leg_typedef::leg_typedef(const std::string& motor1, const std::string& motor2, const std::string& motor3, 
                        const std::string& sensor1, const std::string& sensor2, const std::string& sensor3, 
                        const float& init_angle1, const float& init_angle2, const float& target_length) {
  this->motor1 = new webots::Motor(motor1);
  this->motor2 = new webots::Motor(motor2);
  this->motor3 = new webots::Motor(motor3);
  this->sensor1 = new webots::PositionSensor(sensor1);
  this->sensor2 = new webots::PositionSensor(sensor2);
  this->sensor3 = new webots::PositionSensor(sensor3);
  this->sensor1->enable(time_step);
  this->sensor2->enable(time_step);
  this->sensor3->enable(time_step);
  this->init_angle1 = init_angle1;
  this->init_angle2 = init_angle2;
  this->l1 = 0.18;
  this->l2 = 0.18;
  this->length_pid.init(pid_pos_mode, leg_length_pid, 0, 1);
  this->length_pid.set_target(target_length);
  this->length = { 0 };
  this->theta = { 0 };

  this->f_leg_feedforward = 58.85f;
}
void leg_typedef::zjie(const float& pitch) {
  this->length.now = 2 * this->l1 * sinf(this->angle2 / 2);
  this->phi = this->angle1 + 1.5707963267895 - this->angle2 / 2;

  this->toe_x = this->length.now * cos(this->phi);
  this->toe_y = this->length.now * sin(this->phi);

  float rotation_matrix[2][1];
  rotation_matrix[0][0] = cosf(pitch) * this->toe_x - sinf(pitch) * this->toe_y;
  rotation_matrix[1][0] = sinf(pitch) * this->toe_x + cosf(pitch) * this->toe_y;

  this->theta.now = atan2f(rotation_matrix[0][0], rotation_matrix[1][0]);
   
  this->length.dot = (this->length.now - this->length.last) / time_step * 1000;
  this->length.ddot = (this->length.dot - this->length.last_dot) / time_step * 1000;
  this->length.last_dot = this->length.dot;
  this->length.last = this->length.now;

  this->theta.dot = (this->theta.now - this->theta.last) / time_step * 1000;
  this->theta.ddot = (this->theta.dot - this->theta.last_dot) / time_step * 1000;
  this->theta.last_dot = this->theta.dot;
  this->theta.last = this->theta.now;
}

void leg_typedef::vmc() {
  Eigen::Matrix<float, 2, 2> jacobin_trans;
  Eigen::Matrix<float, 2, 1> t, f;
  jacobin_trans << 0, 1, this->l1* cosf(this->angle2 / 2), -0.5;
  f << this->f_leg, -this->t_joint;
  t = jacobin_trans * f;
  this->t_motor1 = t(0, 0);
  this->t_motor2 = t(1, 0);
}

void leg_typedef::invmc() {
  this->t_motor1_now = this->t_motor1;
  this->t_motor2_now = this->t_motor2;
  
  Eigen::Matrix<float, 2, 2> jacobin_trans;
  Eigen::Matrix<float, 2, 1> t, f;
  float temp = this->l1 * cosf(this->angle2 / 2);
  jacobin_trans << 0.5 / temp, 1 / temp, 1, 0;
  t << this->t_motor1_now, this->t_motor2_now;
  f = jacobin_trans * t;
  this->f_leg_now = f(0, 0);
  this->t_joint_now = -f(1, 0);
}

void leg_typedef::run_motor() {
  if (this->t_motor1 > 17)
    this->t_motor1 = 17;
  if (this->t_motor1 < -17)
    this->t_motor1 = -17;
  if (this->t_motor2 > 17)
    this->t_motor2 = 17;
  if (this->t_motor2 < -17)
    this->t_motor2 = -17;
  if (this->t_motor3 > 3)
    this->t_motor3 = 3;
  if (this->t_motor3 < -3)
    this->t_motor3 = -3;

  this->motor1->setTorque(this->t_motor1);
  this->motor2->setTorque(-this->t_motor2);
  this->motor3->setTorque(-this->t_motor3);
}

void leg_typedef::set_length(const float& length) {
  this->length_pid.set_target(length);
}

void leg_typedef::update() {
  this->angle1 = this->sensor1->getValue() + this->init_angle1;
  this->angle2 = -this->sensor2->getValue() + this->init_angle2;
  this->angle3 = -this->sensor3->getValue();
  this->dis.now = this->angle3 * wheel_r;
  this->dis.dot = (this->dis.now - this->dis.last) / time_step * 1000;
  this->dis.last = this->dis.now;
  this->invmc();
}


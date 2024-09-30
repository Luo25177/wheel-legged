#include "luo_robot.hpp"

float leg_init_angle1 = 0.52359877f;
float leg_init_angle2 = 1.04719755f;
pid_param split_pid_param = {50, 0, 500};

robot_typedef::robot_typedef() {
  this->leg_l = new leg_typedef("left_motor1", "left_motor2", "left_motor3",
                                "left_sensor1", "left_sensor2", "left_sensor3",
                                leg_init_angle1, leg_init_angle2, 0.18f);
  this->leg_r =
      new leg_typedef("right_motor1", "right_motor2", "right_motor3",
                      "right_sensor1", "right_sensor2", "right_sensor3",
                      leg_init_angle1, leg_init_angle2, 0.18f);
  this->leg_vir =
      new leg_typedef("right_motor1", "right_motor2", "right_motor3",
                      "right_sensor1", "right_sensor2", "right_sensor3",
                      leg_init_angle1, leg_init_angle2, 0.18f);
  this->sensor = new sensor_typedef("gyro", "imu", "accel");
  this->fly_flag = false;
  this->split_k_coef << -20.38092261, 52.71883802, -123.2269622, -12.67241015,
      31.19239506, -40.3300934, -12.66792472, -1.665980051, -54.30593255,
      51.84662323, -17.38309216, -20.18800571, 2.508775759, 4.574150597,
      -14.45940275, -14.67885334, -415.8947972, 450.6473363, -185.2707309,
      34.57269028, -19.41376765, 20.94625361, -8.842543632, 1.927947794,
      -5.93831009, 29.97138147, -26.81080522, 14.25171856, -2.638722192,
      3.360204359, -1.195518595, 1.763096091, -263.0349652, 285.0144009,
      -117.1754987, 21.86568922, -178.0546355, 190.6905586, -77.70746776,
      15.0605626, 343.4608746, -327.9068368, 109.940328, 127.6801589,
      19.49042385, -19.03599073, 6.647727147, 2.19795507;

  this->phi_component << -53.9432493324381, 44.2650845282578, -13.4379392846645,
      1.68763610454413;
}

void robot_typedef::update() {
  this->leg_l->update();
  this->leg_r->update();
  this->sensor->update();
  this->leg_l->zjie(this->sensor->pitch.now);
  this->leg_r->zjie(this->sensor->pitch.now);
  this->leg_vir->angle1 = (leg_l->angle1 + leg_r->angle1) / 2;
  this->leg_vir->angle2 = (leg_l->angle2 + leg_r->angle2) / 2;
  this->leg_vir->zjie(this->sensor->pitch.now);
  this->fly_check();
}

void robot_typedef::run() {
  float ll1 = this->leg_l->length.now;
  float ll2 = ll1 * ll1;
  float ll3 = ll2 * ll1;

  float lr1 = this->leg_r->length.now;
  float lr2 = lr1 * lr1;
  float lr3 = lr2 * lr1;

  if (this->fly_flag) {
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        int num = (row * 6) + col;
        if (row == 1 && (col == 0 || col == 1)) {
          this->leg_l->split_k(row, col) = this->split_k_coef(num, 0) * ll3 + this->split_k_coef(num, 1) * ll2 + this->split_k_coef(num, 2) * ll1 + this->split_k_coef(num, 3);
          this->leg_r->split_k(row, col) = this->split_k_coef(num, 0) * lr3 + this->split_k_coef(num, 1) * lr2 + this->split_k_coef(num, 2) * lr1 + this->split_k_coef(num, 3);
        } else {
          this->leg_l->split_k(row, col) = 0;
          this->leg_r->split_k(row, col) = 0;
        }
      }
    }
  } else {
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        int num                      = (row * 6) + col;
        this->leg_l->split_k(row, col) = this->split_k_coef(num, 0) * ll3 + this->split_k_coef(num, 1) * ll2 + this->split_k_coef(num, 2) * ll1 + this->split_k_coef(num, 3);
        this->leg_r->split_k(row, col) = this->split_k_coef(num, 0) * lr3 + this->split_k_coef(num, 1) * lr2 + this->split_k_coef(num, 2) * lr1 + this->split_k_coef(num, 3);
      }
    }
  }

  float l_phi_component = this->phi_component(0, 0) * powf(this->leg_l->length.now, 3)
    + this->phi_component(1, 0) * powf(this->leg_l->length.now, 2)
    + this->phi_component(2, 0) * powf(this->leg_l->length.now, 1)
    + this->phi_component(3, 0);
  float r_phi_component = this->phi_component(0, 0) * powf(this->leg_r->length.now, 3)
    + this->phi_component(1, 0) * powf(this->leg_r->length.now, 2)
    + this->phi_component(2, 0) * powf(this->leg_r->length.now, 1)
    + this->phi_component(3, 0);

  static float x = 0.5;

  this->leg_l->state_split << this->leg_l->theta.now - l_phi_component, this->leg_l->theta.dot, this->leg_l->dis.now, this->leg_l->dis.dot, this->sensor->pitch.now, this->sensor->pitch.dot;
  this->leg_r->state_split << this->leg_r->theta.now - l_phi_component, this->leg_r->theta.dot, this->leg_r->dis.now, this->leg_r->dis.dot, this->sensor->pitch.now, this->sensor->pitch.dot;
  this->expect_split << 0, 0, x, 0, 0, 0;
  this->leg_l->input_split = this->leg_l->split_k * (this->expect_split - this->leg_l->state_split);
  this->leg_r->input_split = this->leg_r->split_k * (this->expect_split - this->leg_r->state_split);
  
  this->leg_l->t_motor3 = this->leg_l->input_split(0, 0);
  this->leg_l->t_joint = this->leg_l->input_split(1, 0);
  this->leg_r->t_motor3 = this->leg_r->input_split(0, 0);
  this->leg_r->t_joint = this->leg_r->input_split(1, 0);

  this->leg_l->f_leg = this->leg_l->f_leg_feedforward;
  this->leg_r->f_leg = this->leg_r->f_leg_feedforward;

  this->leg_l->f_leg += this->leg_l->length_pid.compute(this->leg_l->length.now);
  this->leg_r->f_leg += this->leg_r->length_pid.compute(this->leg_r->length.now);

  this->leg_l->vmc();
  this->leg_r->vmc();
  
  this->leg_l->run_motor();
  this->leg_r->run_motor();
}
void robot_typedef::fly_check() {
  float lpw, rpw, lzwdd, rzwdd, zbdd;

  lpw = this->leg_l->f_leg_now * cos(this->leg_l->theta.now) -
        this->leg_l->t_joint_now * sin(this->leg_l->theta.now) /
            this->leg_l->length.now;
  rpw = this->leg_r->f_leg_now * cos(this->leg_r->theta.now) -
        this->leg_r->t_joint_now * sin(this->leg_r->theta.now) /
            this->leg_r->length.now;

  zbdd = (this->sensor->accel_z * cos(this->sensor->pitch.now) -
          this->sensor->accel_x * sin(this->sensor->pitch.now)) *
             cos(this->sensor->roll.now) +
         this->sensor->accel_y * sin(this->sensor->roll.now);

  lzwdd = zbdd - this->leg_l->length.ddot * cos(this->leg_l->theta.now) +
          2 * this->leg_l->length.dot +
          this->leg_l->theta.dot * sin(this->leg_l->theta.now) +
          this->leg_l->length.now * this->leg_l->theta.ddot *
              sin(this->leg_l->theta.now) +
          this->leg_l->length.now * this->leg_l->theta.dot *
              this->leg_l->theta.dot * cos(this->leg_l->theta.now);
  rzwdd = zbdd - this->leg_r->length.ddot * cos(this->leg_r->theta.now) +
          2 * this->leg_r->length.dot +
          this->leg_r->theta.dot * sin(this->leg_r->theta.now) +
          this->leg_r->length.now * this->leg_r->theta.ddot *
              sin(this->leg_r->theta.now) +
          this->leg_r->length.now * this->leg_r->theta.dot *
              this->leg_r->theta.dot * cos(this->leg_r->theta.now);

  float forcer = rpw + wheel_mass * (gravity - rzwdd);
  float forcel = lpw + wheel_mass * (gravity - lzwdd);

  this->force = (forcer + forcel) / 2;

  if (this->force < 30)
    this->fly_flag = true;
  else
    this->fly_flag = false;
}

void robot_typedef::jump_check() {
  switch (this->jump_phase) {
  case jump_phase_halt:
    break;
  case jump_phase_begin:
    this->leg_l->set_length(0.17f);
    this->leg_r->set_length(0.17f);
    if (this->leg_l->length.now + this->leg_r->length.now <= 0.36f)
      this->jump_phase = jump_phase_kick;
    break;
  case jump_phase_kick:
    this->leg_l->set_length(0.34f);
    this->leg_r->set_length(0.34f);
    if (this->leg_l->length.now + this->leg_r->length.now >= 0.66f)
      this->jump_phase = jump_phase_shrink;
    break;
  case jump_phase_shrink:
    this->leg_l->set_length(0.17f);
    this->leg_r->set_length(0.17f);
    if (this->force > 100)
      this->jump_phase = jump_phase_buffer;
    break;
  case jump_phase_buffer:
    this->leg_l->set_length(0.20f);
    this->leg_r->set_length(0.20f);
    this->jump_phase = jump_phase_halt;
    break;
  case jump_phase_end:
    break;
  default:
    break;
  }
}


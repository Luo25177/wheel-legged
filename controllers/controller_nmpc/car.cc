#include "Car.hh"

#define wheel_r 0.05
#define nmpc_h 30
#define nmpc_step 0.03

Car::Car() {
  tb_motor = new webots::Motor("tb_motor");
  tw_motor = new webots::Motor("tw_motor");

  tb_sensor = new webots::PositionSensor("tb_sensor");
  tw_sensor = new webots::PositionSensor("tw_sensor");

  imu = new webots::InertialUnit("imu");

  gyro = new webots::Gyro("gyro");
  
  tb_sensor->enable(time_step);
  tw_sensor->enable(time_step);
  imu->enable(time_step);
  gyro->enable(time_step);

  tw_torque = 0;
  tb_torque = 0;

  SX X = SX::sym("x", 6);
  SX U = SX::sym("u", 2);

  SX theta = X(0);
  SX theta_dot = X(1);
  SX tw = U(0);
  SX tb = U(1);
  SX numerator =  3.779 * cos(theta) * sin(theta) * pow(theta_dot, 2) + 152.0 * (tw - tb) + 346.1 * tw * cos(theta) + -325.7 * sin(theta);
  SX denominator = 143.009 * pow(cos(theta), 2) + 349.409 * pow(sin(theta), 2);
  SX theta_ddot = -54.61 * numerator / denominator;
  numerator = 10109.1 * tw + 110.419 * pow(theta_dot, 2) * sin(theta) - 5620.0 * cos(theta) * sin(theta) +  2623.0 * (tw - tb) * cos(theta);
  denominator = 143.009 * pow(cos(theta), 2) + 349.409 * pow(sin(theta), 2);
  SX x_ddot = 0.3603 * numerator / denominator;
  SX phi_ddot = 370.8 * tb;

  SX X_dot = vertcat(
    X(1),
    theta_ddot,
    X(3),
    x_ddot,
    X(5),
    phi_ddot
    );

  Function dynamics = Function("pendulum", std::vector<SX>{X, U}, std::vector<SX>{X_dot}, std::vector<std::string>{"x", "u"}, std::vector<std::string>{"xdot"});
  
  DM Q = DM::zeros(6, 6);
  DM R = DM::zeros(2, 2);

  Q(0, 0) = 100;
  Q(1, 1) = 10;
  Q(2, 2) = 500;
  Q(3, 3) = 100;
  Q(4, 4) = 5000;
  Q(5, 5) = 10;

  R(0, 0) = 0.0005;
  R(1, 1) = 0.0008;

  DM u_min = DM::zeros(2);
  DM u_max = DM::zeros(2);
  u_min(0) = -20;
  u_min(1) = -20;
  u_max(0) = 20;
  u_max(1) = 20;

  nmpc.setup(nmpc_h, nmpc_step, 6, 2, dynamics, Q, R, u_min, u_max);
}

Car::~Car() {
}

void Car::update() {
  phi.now = imu->getRollPitchYaw()[1];
  phi.dot = gyro->getValues()[2];

  double x_last = x.now;
  x.now = tw_sensor->getValue() * wheel_r;
  x.dot = (x.now - x_last) / delta_time;

  double tb_last = tb_status.now;
  tb_status.now = tb_sensor->getValue();
  tb_status.dot = (tb_status.now - tb_last) / delta_time;

  theta.now = tb_status.now - phi.now;
  theta.dot = tb_status.dot - phi.dot;
}

void Car::run_nmpc() {
  DM x0 = DM({ theta.now, theta.dot, x.now, x.dot, phi.now, phi.dot });
  DM x_ref = DM({ 0, 0, 0.5, 0, 0, 0 });

  DM out = nmpc.solve(x0, x_ref);

  tw_torque = static_cast<double>(out(0, 0));
  tb_torque = static_cast<double>(out(1, 0));

  tw_motor->setTorque(tw_torque);
  tb_motor->setTorque(tb_torque);
}

void Car::run_lqr() {
  Eigen::Matrix<double, 6, 1> X;
  Eigen::Matrix<double, 6, 1> X_ref;
  Eigen::Matrix<double, 2, 1> U;
  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> K;

  X << theta.now, theta.dot, x.now, x.dot, phi.now, phi.dot;

  X_ref << 0, 0, 0.5, 0, 0, 0;
  K << -31.0466193560277, -4.91164057790086, -22.1082932397901, -17.1753357493249, 10.5940407887983, 0.39526225542489,
    8.5766925997434, 1.37953615381932, 6.70025970346512, 5.05603520157204, 139.825123633277, 2.1029352916582;

  U = K * (X_ref - X);
  
  double tb_torque = U(1, 0);
  double tw_torque = U(0, 0);
  
  //cout << tb_torque << "\t" << tw_torque << endl;

  tw_motor->setTorque(tw_torque);
  tb_motor->setTorque(tb_torque);
}

void Car::data_show(FILE* fp) {
  fprintf(fp, "%f\t%f\t%f\t%f\%f\t%f\n", theta.now, theta.dot, x.now, x.dot, phi.now, phi.dot);
}

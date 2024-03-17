#include "Leg.hpp"

PIDParam legPIDParam = { 2000, 50, 35000 };

Leg::Leg(const LEGDIR& _dir) {
  switch (_dir) {
    case LEFT:
      this->jointB = new Joint(1, "EncoderBL", "MotorBL", InitAngle4, 22);
      this->jointF = new Joint(-1, "EncoderFL", "MotorFL", InitAngle1, 22);
      this->wheel  = new Wheel(1, "EncoderWheelL", "MotorWheelL", 0, 10);
      break;
    case RIGHT:
      this->jointB = new Joint(1, "EncoderBR", "MotorBR", InitAngle4, 22);
      this->jointF = new Joint(-1, "EncoderFR", "MotorFR", InitAngle1, 22);
      this->wheel  = new Wheel(1, "EncoderWheelR", "MotorWheelR", 0, 10);
      break;
  }
  this->theta = DataStruct();
  this->L0    = DataStruct();
  this->dis   = DataStruct();
  this->L0PID.init(PIDPOSMODE, legPIDParam, 0, 1);
  this->L0PID.setTarget(0.3f);
}

Leg::~Leg() {
  delete this->jointB;
  delete this->jointF;
  delete this->wheel;
}

void Leg::update() {
  this->jointB->update();
  this->jointF->update();
  this->wheel->update();
  this->INVMC();
  this->angle1  = jointF->posRead;
  this->angle4  = jointB->posRead;
  this->dis.now = this->wheel->posRead * WHEELR;
  this->dis.dot = this->wheel->velocityRead * WHEELR;
}

void Leg::run() {
  this->jointB->run();
  this->jointF->run();
  this->wheel->run();
}

void Leg::zjie(const float& _pitch) {
  this->xb     = l1 * cos(this->angle1) - l5 / 2;
  this->yb     = l1 * sin(this->angle1);
  this->xd     = l5 / 2 + l4 * cos(this->angle4);
  this->yd     = l4 * sin(this->angle4);
  float lbd    = sqrt(pow((this->xd - this->xb), 2) + pow((this->yd - this->yb), 2));
  float A0     = 2 * l2 * (this->xd - this->xb);
  float B0     = 2 * l2 * (this->yd - this->yb);
  float C0     = pow(l2, 2) + pow(lbd, 2) - pow(l3, 2);
  float D0     = pow(l3, 2) + pow(lbd, 2) - pow(l2, 2);
  this->angle2 = 2 * atan2((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))), (A0 + C0));
  this->angle3 = PI - 2 * atan2((-B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(D0, 2))), (A0 + D0));
  this->xc     = this->xb + l2 * cos(this->angle2);
  this->yc     = this->yb + l2 * sin(this->angle2);

  this->L0.now = sqrt(pow(this->xc, 2) + pow(this->yc, 2));
  this->angle0 = atan2(this->yc, this->xc);

  // 乘以_pitch的旋转矩阵
  float cor_XY_then[2][1];
  cor_XY_then[0][0]   = cos(_pitch) * this->xc - sin(_pitch) * this->yc;
  cor_XY_then[1][0]   = sin(_pitch) * this->xc + cos(_pitch) * this->yc;
  this->theta.now     = atan2(cor_XY_then[0][0], cor_XY_then[1][0]);

  this->L0.dot        = (this->L0.now - this->L0.last) / DT;
  this->L0.ddot       = (this->L0.dot - this->L0.lastdot) / DT;
  this->L0.last       = this->L0.now;
  this->L0.lastdot    = this->L0.dot;
  this->theta.dot     = (this->theta.now - this->theta.last) / DT;
  this->theta.ddot    = (this->theta.dot - this->theta.lastdot) / DT;
  this->theta.last    = this->theta.now;
  this->theta.lastdot = this->theta.dot;
}

void Leg::zjie(const float& _pitch, const float& _angle1, const float _angle4) {
  this->angle1 = _angle1;
  this->angle4 = _angle4;
  this->xb     = l1 * cos(this->angle1) - l5 / 2;
  this->yb     = l1 * sin(this->angle1);
  this->xd     = l5 / 2 + l4 * cos(this->angle4);
  this->yd     = l4 * sin(this->angle4);
  float lbd    = sqrt(pow((this->xd - this->xb), 2) + pow((this->yd - this->yb), 2));
  float A0     = 2 * l2 * (this->xd - this->xb);
  float B0     = 2 * l2 * (this->yd - this->yb);
  float C0     = pow(l2, 2) + pow(lbd, 2) - pow(l3, 2);
  float D0     = pow(l3, 2) + pow(lbd, 2) - pow(l2, 2);
  this->angle2 = 2 * atan2((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))), (A0 + C0));
  this->angle3 = PI - 2 * atan2((-B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(D0, 2))), (A0 + D0));
  this->xc     = this->xb + l2 * cos(this->angle2);
  this->yc     = this->yb + l2 * sin(this->angle2);

  this->L0.now = sqrt(pow(this->xc, 2) + pow(this->yc, 2));
  this->angle0 = atan2(this->yc, this->xc);

  // 乘以_pitch的旋转矩阵
  float cor_XY_then[2][1];
  cor_XY_then[0][0]   = cos(_pitch) * this->xc - sin(_pitch) * this->yc;
  cor_XY_then[1][0]   = sin(_pitch) * this->xc + cos(_pitch) * this->yc;
  this->theta.now     = atan2(cor_XY_then[0][0], cor_XY_then[1][0]);

  this->L0.dot        = (this->L0.now - this->L0.last) / DT;
  this->L0.ddot       = (this->L0.dot - this->L0.lastdot) / DT;
  this->L0.last       = this->L0.now;
  this->L0.lastdot    = this->L0.dot;
  this->theta.dot     = (this->theta.now - this->theta.last) / DT;
  this->theta.ddot    = (this->theta.dot - this->theta.lastdot) / DT;
  this->theta.last    = this->theta.now;
  this->theta.lastdot = this->theta.dot;
}

void Leg::njie(const float& _xc, const float& _yc) {
  float A, B, C;

  A               = 2 * (xc + l5 / 2) * l1;
  B               = 2 * yc * l1;
  C               = (xc + l5 / 2) * (xc + l5 / 2) + yc * yc + l1 * l1 - l2 * l2;
  this->angle1set = 2 * atan2((B + sqrt(A * A + B * B - C * C)), (A + C));
  A               = 2 * l4 * (xc - l5 / 2);
  B               = 2 * l4 * yc;
  C               = (xc - l5 / 2) * (xc - l5 / 2) + l4 * l4 + yc * yc - l3 * l3;
  this->angle4set = 2 * atan2((B - sqrt(A * A + B * B - C * C)), (A + C));
  if (this->angle4set < 0)
    this->angle4set += 2 * PI;
}

void Leg::VMC() {
  float                      A = l1 * sin(angle1 - angle2) / sin(angle2 - angle3);
  float                      B = l4 * sin(angle3 - angle4) / sin(angle2 - angle3);
  Eigen::Matrix<float, 2, 2> trans;
  trans << -A * cos(theta.now + angle3), 
					 A * sin(theta.now + angle3) / L0.now, 
           -B * cos(theta.now + angle2), 
           B * sin(theta.now + angle2) / L0.now;

  this->jointF->setTorque(-trans(0, 0) * this->Fset - trans(0, 1) * this->Tbset);
  this->jointB->setTorque(-trans(1, 0) * this->Fset - trans(1, 1) * this->Tbset);
  this->wheel->setTorque(this->Twset);
}

void Leg::INVMC() {
  float A = l1 * sin(angle1 - angle2) / sin(angle2 - angle3);
  float B = l4 * sin(angle3 - angle4) / sin(angle2 - angle3);
  A       = A * cos(angle2 + theta.now) * sin(angle3 + theta.now) - A * cos(angle3 + theta.now) * sin(angle2 + theta.now);
  B       = B * cos(angle2 + theta.now) * sin(angle3 + theta.now) - B * cos(angle3 + theta.now) * sin(angle2 + theta.now);

  Eigen::Matrix<float, 2, 2> trans;
  trans << sin(angle2 + theta.now) / A, -sin(angle3 + theta.now) / B, L0.now * cos(angle2 + theta.now) / B, -L0.now * cos(angle3 + theta.now) / B;

  Fnow  = -trans(0, 0) * jointF->torqueRead - trans(0, 1) * jointB->torqueRead;
  Tbnow = -trans(1, 0) * jointF->torqueRead - trans(1, 1) * jointB->torqueRead;
}

void Leg::splitCompute(const float& _pitch, const float& _pitchdot, const Eigen::Matrix<float, 12, 4>& _kcoeff) {
}

#include "leg.h"

//----
// @brief 初始化
//
// @param leg
// @param dir 腿的方向
//----
void LegInit(Leg* leg, int dir, Zdrive* wheel, Tmotor* front, Tmotor* behind) {
  leg->dir = dir;

  PidInit(&leg->L0pid, 1000, 2, 9500, 0, 1000, PIDPOS);

  DataStructInit(&leg->dis, 0, 0, 0, 0);
  DataStructInit(&leg->L0, 0, 0, 0, 0);
  DataStructInit(&leg->theta, 0, 0, 0, 0);

  leg->wheel                = wheel;
  leg->front                = front;
  leg->behind               = behind;

  leg->Fset                 = 0;
  leg->Tpset                = 0;
  leg->TFset                = 0;
  leg->TBset                = 0;
  leg->TWheelset            = 0;
  leg->timer                = 0;
  leg->normalforce          = 0;
  // 电机绑定
  leg->front->initSetAngle  = FrontAngleInit;
  leg->behind->initSetAngle = BehindAngleInit;

  InputInit(&leg->X);
  InputInit(&leg->Xd);
  OutputInit(&leg->U);
}

//----
// @brief 腿部状态量更新
//
// @param leg
//----
void LegUpdate(Leg* leg) {
  leg->angle1    = (float) (leg->front->initSetAngle - (leg->front->real.angleRad * leg->dir));
  leg->angle4    = (float) (leg->behind->initSetAngle - (leg->behind->real.angleRad * leg->dir));
  leg->dis.now   = (float) leg->wheel->real.angleRad * WHEELR * leg->dir;
  leg->dis.dot   = (float) leg->wheel->real.velocity * WHEELR * leg->dir;
  leg->TFnow     = (float) leg->front->real.torque * leg->dir;
  leg->TBnow     = (float) leg->behind->real.torque * leg->dir;
  leg->TWheelnow = (float) leg->wheel->real.torque * leg->dir;
  INVMC(leg);
}

//----
// @brief 计算L0和angle0的原值和导数值
//
// @param leg
// @param pitch 俯仰角 用于坐标系的转换
//----
void Zjie(Leg* leg, float pitch) {
  leg->xb     = l1 * cos(leg->angle1) - l5 / 2;
  leg->yb     = l1 * sin(leg->angle1);
  leg->xd     = l5 / 2 + l4 * cos(leg->angle4);
  leg->yd     = l4 * sin(leg->angle4);
  float lbd   = sqrt(powf((leg->xd - leg->xb), 2) + powf((leg->yd - leg->yb), 2));
  float A0    = 2 * l2 * (leg->xd - leg->xb);
  float B0    = 2 * l2 * (leg->yd - leg->yb);
  float C0    = powf(l2, 2) + powf(lbd, 2) - powf(l3, 2);
  float D0    = powf(l3, 2) + powf(lbd, 2) - powf(l2, 2);
  leg->angle2 = 2 * atan2f((B0 + sqrt(powf(A0, 2) + powf(B0, 2) - powf(C0, 2))), (A0 + C0));
  leg->angle3 = PI - 2 * atan2f((-B0 + sqrt(powf(A0, 2) + powf(B0, 2) - powf(D0, 2))), (A0 + D0));
  leg->xc     = leg->xb + l2 * cos(leg->angle2);
  leg->yc     = leg->yb + l2 * sin(leg->angle2);
  leg->L0.now = sqrt(powf(leg->xc, 2) + powf(leg->yc, 2));
  leg->angle0 = atan2(leg->yc, leg->xc);

  // 乘以pitch的旋转矩阵
  float cor_XY_then[2];
  cor_XY_then[0] = cos(pitch) * leg->xc - sin(pitch) * leg->yc;
  cor_XY_then[1] = sin(pitch) * leg->xc + cos(pitch) * leg->yc;
  leg->theta.now = atan2f(cor_XY_then[0], cor_XY_then[1]);

  float dt       = 10000 / (float) (GlobalTimer - leg->timer);
  leg->timer     = GlobalTimer;

  if (dt > 0) {
    leg->L0.dot        = (leg->L0.now - leg->L0.last) * dt;
    leg->L0.ddot       = (leg->L0.dot - leg->L0.lastdot) * dt;
    leg->L0.last       = leg->L0.now;
    leg->L0.lastdot    = leg->L0.dot;
    leg->theta.dot     = (leg->theta.now - leg->theta.last) * dt;
    leg->theta.ddot    = (leg->theta.dot - leg->theta.lastdot) * dt;
    leg->theta.last    = leg->theta.now;
    leg->theta.lastdot = leg->theta.dot;
  }
}

//----
// @brief 腿部结构逆解，用于解算出脚端落点的对应的电机角度
//
// @param leg
// @param xc
// @param yc
//----
void Njie(Leg* leg, float xc, float yc) {
  float A, B, C;

  A              = 2 * (xc + l5 / 2) * l1;
  B              = 2 * yc * l1;
  C              = (xc + l5 / 2) * (xc + l5 / 2) + yc * yc + l1 * l1 - l2 * l2;
  leg->angle1set = 2 * atan2f((B + sqrt(A * A + B * B - C * C)), (A + C));
  A              = 2 * l4 * (xc - l5 / 2);
  B              = 2 * l4 * yc;
  C              = (xc - l5 / 2) * (xc - l5 / 2) + l4 * l4 + yc * yc - l3 * l3;
  leg->angle4set = 2 * atan2f((B - sqrt(A * A + B * B - C * C)), (A + C));
  if (leg->angle4set < 0)
    leg->angle4set += 2 * PI;
}

//----
// @brief VMC 解算，虚拟力->实际电机扭矩
//
// @param leg
//----
void VMC(Leg* leg) {
  float trans[2][2] = { l1 * sin(leg->angle0 - leg->angle3) * sin(leg->angle1 - leg->angle2) / sin(leg->angle2 - leg->angle3),
                        l1 * cos(leg->angle0 - leg->angle3) * sin(leg->angle1 - leg->angle2) / (leg->L0.now * sin(leg->angle2 - leg->angle3)),
                        l4 * sin(leg->angle0 - leg->angle2) * sin(leg->angle3 - leg->angle4) / sin(leg->angle2 - leg->angle3),
                        l4 * cos(leg->angle0 - leg->angle2) * sin(leg->angle3 - leg->angle4) / (leg->L0.now * sin(leg->angle2 - leg->angle3)) };
  leg->TFset        = trans[0][0] * leg->Fset - trans[0][1] * leg->Tpset;
  leg->TBset        = trans[1][0] * leg->Fset - trans[1][1] * leg->Tpset;
}

//----
// @brief 逆向VMC解算 实际电机扭矩->虚拟力
//
// @param leg
//----
void INVMC(Leg* leg) {
  float A           = sin(leg->angle2 - leg->angle3) / (l1 * cos(leg->angle0 - leg->angle2) * sin(leg->angle0 - leg->angle3) * sin(leg->angle1 - leg->angle2) - l1 * cos(leg->angle0 - leg->angle3) * sin(leg->angle0 - leg->angle2) * sin(leg->angle1 - leg->angle2));
  float B           = sin(leg->angle2 - leg->angle3) / (l4 * cos(leg->angle0 - leg->angle2) * sin(leg->angle0 - leg->angle3) * sin(leg->angle3 - leg->angle4) - l4 * cos(leg->angle0 - leg->angle3) * sin(leg->angle0 - leg->angle2) * sin(leg->angle3 - leg->angle4));

  float trans[2][2] = { cos(leg->angle0 - leg->angle2) * A, -cos(leg->angle0 - leg->angle3) * B, -leg->L0.now * sin(leg->angle0 - leg->angle2) * A, leg->L0.now * sin(leg->angle0 - leg->angle3) * B };

  leg->Fnow         = trans[0][0] * leg->TFnow + trans[0][1] * leg->TBnow;
  leg->Tpnow        = -trans[1][0] * leg->TFnow - trans[1][1] * leg->TBnow;
}

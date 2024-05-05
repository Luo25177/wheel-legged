#include "EKF.h"

void ExternKalmanFilterInit(ExternKalmanFilter *filter, float _Fk, float _Hk, float _Qk, float _Rk, float _JF, float _JH) {
  filter->Fk = _Fk;
  filter->Hk = _Hk;
  filter->Qk = _Qk;
  filter->Rk = _Rk;
  filter->JF = _JF;
  filter->JH = _JH;
}

float ExternKalmanFilterRun(ExternKalmanFilter *filter, float input) {
  float xkk_1 = filter->Fk * filter->Xkk;
  float pkk_1 = filter->JF * filter->Pkk * filter->JF + filter->Qk;
  float yk = input - filter->Hk * xkk_1;
  float Kk = pkk_1 * filter->JH / (filter->JH * pkk_1 * filter->Hk + filter->Rk);
  filter->Xkk = xkk_1 + Kk * yk;
  filter->Pkk = (1 - Kk * filter->JH) * pkk_1;
  return filter->Xkk;
}
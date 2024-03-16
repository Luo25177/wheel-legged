#include "KF.h"

void KalmanFilterInit(KalmanFilter* filter, float _Fk, float _Hk, float _Qk, float _Rk) {
  filter->Xkk = 0;
  filter->Pkk = 0;
  filter->Fk  = _Fk;
  filter->Hk  = _Hk;
  filter->Qk  = _Qk;
  filter->Rk  = _Rk;
}

float KalmanFilterRun(KalmanFilter* filter, float _input) {
  float Xkk_1 = filter->Fk * filter->Xkk;
  float Pkk_1 = filter->Fk * filter->Pkk * filter->Fk + filter->Qk;
  float Kk    = Pkk_1 * filter->Hk / (filter->Hk * Pkk_1 * filter->Hk + filter->Rk);
  filter->Xkk = Xkk_1 + Kk * (_input - filter->Hk * Xkk_1);
  filter->Pkk = (1 - Kk * filter->Hk) * Pkk_1;
  return filter->Xkk;
}

void KalmanFilterMatrixInit(KalmanFilterMatrix* filter, const int n, const int p, float* _Fk, float* _Hk, float* _Qk, float* _Rk) {
  filter->Xkk_data = (float*) malloc(n * 1 * 4);
  filter->Pkk_data = (float*) malloc(n * n * 4);
  filter->Fkk_data = (float*) malloc(n * n * 4);
  filter->Rkk_data = (float*) malloc(p * p * 4);
  filter->Qkk_data = (float*) malloc(n * n * 4);
  filter->Hkk_data = (float*) malloc(p * n * 4);

  arm_mat_init_f32(&filter->Xkk, n, 1, filter->Xkk_data);
  arm_mat_init_f32(&filter->Pkk, n, n, filter->Pkk_data);
  arm_mat_init_f32(&filter->Fk, n, n, filter->Fkk_data);
  arm_mat_init_f32(&filter->Hk, p, n, filter->Hkk_data);
  arm_mat_init_f32(&filter->Qk, n, n, filter->Qkk_data);
  arm_mat_init_f32(&filter->Rk, p, p, filter->Rkk_data);
}

arm_matrix_instance_f32 KalmanFilterMatrixRun(KalmanFilterMatrix* filter, arm_matrix_instance_f32 _input) {
  // 预测
  unsigned short          n = filter->Xkk.numRows;
  unsigned short          p = filter->Rk.numRows;
  float                   _xkk_1[n * 1];
  float                   _pkk_1[n * n];
  arm_matrix_instance_f32 Xkk_1;
  arm_matrix_instance_f32 Pkk_1;
  arm_mat_init_f32(&Xkk_1, n, 1, _xkk_1);
  arm_mat_init_f32(&Pkk_1, n, n, _pkk_1);

  arm_mat_mult_f32(&filter->Fk, &filter->Xkk, &Xkk_1);

  arm_matrix_instance_f32 FkT;
  float                   _FkT[n * n];
  arm_mat_init_f32(&FkT, n, n, _FkT);

  arm_mat_trans_f32(&filter->Fk, &FkT);

  arm_matrix_instance_f32 temp;
  float                   _temp[n * n];
  arm_mat_init_f32(&temp, n, n, _temp);

  arm_mat_mult_f32(&filter->Fk, &filter->Pkk, &Pkk_1);
  arm_mat_mult_f32(&Pkk_1, &FkT, &temp);
  arm_mat_add_f32(&temp, &filter->Qk, &Pkk_1);

  // 更新
  arm_matrix_instance_f32 HkT;
  float                   _HkT[n * p];
  arm_mat_init_f32(&HkT, n, p, _HkT);
  arm_mat_trans_f32(&filter->Hk, &HkT);

  arm_matrix_instance_f32 y;
  float                   _y[p * 1];
  arm_mat_init_f32(&temp, n, n, _temp);

  float _temp1[p * 1];
  arm_mat_init_f32(&temp, p, 1, _temp1);

  arm_mat_mult_f32(&filter->Hk, &Xkk_1, &temp);
  arm_mat_sub_f32(&_input, &temp, &y);

  float _temp2[p * p];
  arm_mat_init_f32(&temp, p, p, _temp2);

  float                   _sk[p * p];
  arm_matrix_instance_f32 Sk;
  arm_mat_init_f32(&Sk, p, p, _sk);

  float                   _temp3[p * n];
  arm_matrix_instance_f32 temp1;
  arm_mat_init_f32(&temp1, p, n, _temp3);

  arm_mat_mult_f32(&filter->Hk, &Pkk_1, &temp1);
  arm_mat_mult_f32(&temp1, &HkT, &temp);
  arm_mat_add_f32(&temp, &filter->Rk, &Sk);

  float                   reverse_sk[p * p];
  arm_matrix_instance_f32 RSk;
  arm_mat_init_f32(&RSk, p, p, reverse_sk);
  arm_mat_inverse_f32(&Sk, &RSk);

  float _temp4[n * p];
  arm_mat_init_f32(&temp, n, p, _temp4);

  float                   _kk[n * p];
  arm_matrix_instance_f32 Kk;
  arm_mat_init_f32(&Kk, n, p, _kk);

  arm_mat_mult_f32(&Pkk_1, &HkT, &temp);
  arm_mat_mult_f32(&temp, &RSk, &Kk);

  float _temp5[n * 1];
  arm_mat_init_f32(&temp, n, 1, _temp5);

  arm_mat_mult_f32(&Kk, &y, &temp);
  arm_mat_add_f32(&Xkk_1, &temp, &filter->Xkk);

  arm_mat_init_f32(&temp, n, 1, _temp);
  arm_mat_mult_f32(&Kk, &filter->Hk, &temp);
  float Idata[n * n];
  for (int i = 0, j = 0; i < n; ++i, j += n)
    Idata[j + i] = 1;
  arm_matrix_instance_f32 I;
  arm_mat_init_f32(&i, n, n, Idata);
  float _temp6[n * n];
  arm_mat_init_f32(&temp1, n, n, _temp6);

  arm_mat_sub_f32(&I, &temp, &temp1);
  arm_mat_mult_f32(&temp1, &Pkk_1, &filter->Pkk);
  return filter->Xkk;
}

#pragma once
#include "arm_math.h"
#include "stdlib.h"
#include "string.h"

typedef struct {
  float Xkk;  // 预测值
  float Pkk;  // 后验估计误差协方差矩阵
  float Fk;   // 状态变换矩阵
  float Rk;   // 测量噪声协方差矩阵
  float Qk;   // 过程噪声协方差矩阵
  float Hk;   // 观测矩阵
} KalmanFilter;

void  KalmanFilterInit(KalmanFilter* filter, float _Fk, float _Hk, float _Qk, float _Rk);
float KalmanFilterRun(KalmanFilter* filter, float _input);

typedef struct {
  float* Xkk_data;
  float* Pkk_data;
  float* Fkk_data;
  float* Rkk_data;
  float* Qkk_data;
  float* Hkk_data;

  arm_matrix_instance_f32 Xkk;  // 预测值 n x 1
  arm_matrix_instance_f32 Pkk;  // 后验估计误差协方差矩阵 n x n
  arm_matrix_instance_f32 Fk;   // 状态变换矩阵 n x n
  arm_matrix_instance_f32 Rk;   // 测量噪声协方差矩阵 p x p
  arm_matrix_instance_f32 Qk;   // 过程噪声协方差矩阵 n x n
  arm_matrix_instance_f32 Hk;   // 观测矩阵 p x n
} KalmanFilterMatrix;

void                    KalmanFilterMatrixInit(KalmanFilterMatrix* filter, int n, int p, float* _Fk, float* _Hk, float* _Qk, float* _Rk);
arm_matrix_instance_f32 KalmanFilterMatrixRun(KalmanFilterMatrix* filter, arm_matrix_instance_f32 _input);
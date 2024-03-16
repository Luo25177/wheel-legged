#pragma once

typedef struct {
  float Xkk;  // 预测值
  float Pkk;  // 后验估计误差协方差矩阵
  float Fk;   // 状态变换矩阵
  float JF;
  float Rk;  // 测量噪声协方差矩阵
  float Qk;  // 过程噪声协方差矩阵
  float Hk;  // 观测矩阵
  float JH;
} ExternKalmanFilter;

void  ExternKalmanFilterInit(ExternKalmanFilter* filter, float _Fk, float _Hk, float _Qk, float _Rk, float _JF, float _JH);
float ExternKalmanFilterRun(ExternKalmanFilter* filter, float input);

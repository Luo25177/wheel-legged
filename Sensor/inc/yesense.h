//----
// @file yesense.h
// @author mask <beloved25177@126.com>
// @brief
// @version 1.0
// @date 2023-11-13
//
// @copyright Copyright (c) 2023
//
//----
#pragma once

#include "DataStruct.h"
#include "mymath.h"
#include "tim.h"

#include <stdbool.h>

typedef struct {
  bool init;
  float timer;

  float accelx;// m/s2
  float accely;
  float accelz;

  float magx;
  float magy;
  float magz;

  float rawMagx; /*uinit: mGauss*/
  float rawMagy;
  float rawMagz;

  DataStruct pitch;// deg
  DataStruct roll;
  DataStruct yaw;

  float quaternion_data0;
  float quaternion_data1;
  float quaternion_data2;
  float quaternion_data3;

} Yesense;

void yesenseReceiveHandler(Yesense *yesense, u8 temp);
void yesenseInit(Yesense *yesense);

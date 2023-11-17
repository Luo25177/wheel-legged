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

#include "tim.h"
#include <stdbool.h>
#include "datastruct.h"

typedef struct {
	float accelx; // m/s2
	float accely;
	float accelz;

	float magx;
	float magy;
	float magz;

	float rawMagx;		/*uinit: mGauss*/
	float rawMagy;
	float rawMagz;
	
  datastruct pitch; // deg
  datastruct roll;
  datastruct yaw;
	
	float quaternion_data0;
	float quaternion_data1;	
	float quaternion_data2;
	float quaternion_data3;

  unsigned char data[98];
  short datalen;
  bool init;
}Yesense;

int yesenseAnalyze(Yesense* yesense, unsigned char *data, short len);
void yesenseInit(Yesense* yesense);


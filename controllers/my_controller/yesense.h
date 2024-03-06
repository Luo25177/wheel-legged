#pragma once

#include "DataStruct.h"
#include "mymath.h"
#include "robotparam.h"

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>

typedef struct {
	WbDeviceTag gps;
	WbDeviceTag imu;
	WbDeviceTag gyro;
	WbDeviceTag accel;
	WbDeviceTag camera;

	float accelx;
	float accely;
	float accelz;
	float z;

	DataStruct pitch;
	DataStruct roll;
	DataStruct yaw;
	DataStruct x;
	DataStruct y;

	bool	gpsinit;
	float initx;
	float inity;
	float X;
	float V;
} Yesense;

void yesenseUpdata(Yesense* yesense);
void yesenseInit(Yesense* yesense);

#include "yesense.h"

void yesenseInit(Yesense* yesense) {
	yesense->imu = wb_robot_get_device("imu");
	yesense->gyro = wb_robot_get_device("gyro");
	yesense->accel = wb_robot_get_device("accel");
	yesense->gps = wb_robot_get_device("gps");

	
	wb_inertial_unit_enable(yesense->imu, timestep);
	wb_gps_enable(yesense->gps, timestep);
	wb_accelerometer_enable(yesense->accel, timestep);
	wb_gyro_enable(yesense->gyro, timestep);

	datastructInit(&yesense->pitch, 0, 0, 0, 0);
	datastructInit(&yesense->roll, 0, 0, 0, 0);
	datastructInit(&yesense->yaw, 0, 0, 0, 0);
}

void yesenseUpdata(Yesense* yesense) {
	yesense->pitch.now = -wb_inertial_unit_get_roll_pitch_yaw(yesense->imu)[0];
	yesense->pitch.dot = -wb_gyro_get_values(yesense->gyro)[0];
	yesense->roll.now = wb_inertial_unit_get_roll_pitch_yaw(yesense->imu)[1];
	yesense->roll.dot = wb_gyro_get_values(yesense->gyro)[2];
	yesense->yaw.now = wb_inertial_unit_get_roll_pitch_yaw(yesense->imu)[2];
	yesense->yaw.dot = wb_gyro_get_values(yesense->gyro)[1];
	yesense->accelx = wb_accelerometer_get_values(yesense->accel)[2];
	yesense->accely = wb_accelerometer_get_values(yesense->accel)[0];
	yesense->accelz = wb_accelerometer_get_values(yesense->accel)[1];
}

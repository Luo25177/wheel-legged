#include "yesense.h"

void yesenseInit(Yesense* yesense) {
	yesense->imu		= wb_robot_get_device("imu");
	yesense->gyro		= wb_robot_get_device("gyro");
	yesense->accel	= wb_robot_get_device("accel");
	yesense->gps		= wb_robot_get_device("gps");
	yesense->camera = wb_robot_get_device("camera");

	wb_inertial_unit_enable(yesense->imu, timestep);
	wb_gps_enable(yesense->gps, timestep);
	wb_accelerometer_enable(yesense->accel, timestep);
	wb_gyro_enable(yesense->gyro, timestep);
	wb_camera_enable(yesense->camera, timestep);

	datastructInit(&yesense->pitch, 0, 0, 0, 0);
	datastructInit(&yesense->roll, 0, 0, 0, 0);
	datastructInit(&yesense->yaw, 0, 0, 0, 0);
	yesense->gpsinit = false;
}

void yesenseUpdata(Yesense* yesense) {
	yesense->pitch.now = -wb_inertial_unit_get_roll_pitch_yaw(yesense->imu)[0];
	yesense->pitch.dot = -wb_gyro_get_values(yesense->gyro)[0];
	yesense->roll.now	 = wb_inertial_unit_get_roll_pitch_yaw(yesense->imu)[1];
	yesense->roll.dot	 = wb_gyro_get_values(yesense->gyro)[2];
	float yaw_get			 = wb_inertial_unit_get_roll_pitch_yaw(yesense->imu)[2];
	if (yaw_get - yesense->yaw.last > 1.5 * PI)
		yesense->yaw.now += yaw_get - yesense->yaw.last - 2 * PI;
	else if (yaw_get - yesense->yaw.last < -1.5 * PI)
		yesense->yaw.now += yaw_get - yesense->yaw.last + 2 * PI;
	else
		yesense->yaw.now += yaw_get - yesense->yaw.last;
	yesense->yaw.last = yesense->yaw.now;

	yesense->yaw.dot	= wb_gyro_get_values(yesense->gyro)[1];
	yesense->accelx		= wb_accelerometer_get_values(yesense->accel)[2];
	yesense->accely		= wb_accelerometer_get_values(yesense->accel)[0];
	yesense->accelz		= wb_accelerometer_get_values(yesense->accel)[1];

	if (!yesense->gpsinit) {
		yesense->initx	 = wb_gps_get_values(yesense->gps)[2];
		yesense->inity	 = wb_gps_get_values(yesense->gps)[0];
		yesense->gpsinit = true;
	}

	yesense->z			= wb_gps_get_values(yesense->gps)[1];
	yesense->y.now	= wb_gps_get_values(yesense->gps)[0] - yesense->inity;
	yesense->x.now	= wb_gps_get_values(yesense->gps)[2] - yesense->initx;

	yesense->x.dot	= (yesense->x.now - yesense->x.last) / dt;
	yesense->x.last = yesense->x.now;
	yesense->y.dot	= (yesense->y.now - yesense->y.last) / dt;
	yesense->y.last = yesense->y.now;
}

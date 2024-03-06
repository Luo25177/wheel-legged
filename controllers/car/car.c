#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/position_sensor.h>
#include "ADRC2.h"

#define TIME_STEP 10

double x = 0;
double lastx = 0;
double xdot = 0;
double sita = 0;
double lastsita = 0;
double sitadot = 0;
double fai = 0;
double faidot = 0;

double alpha = 0;

double xexpect = 0;
double xdotexpect = 0;
double sitaexpect = 0;
double sitadotexpect = 0;
double faiexpect = 0;
double faidotexpect = 0;
double Twout;
double Tpout;

void Hinfinty();
void LQR();
void ADRC();

int main(int argc, char **argv) {
  wb_robot_init();
  
  WbDeviceTag motorT = wb_robot_get_device("Tmotor");
  WbDeviceTag sensorT = wb_robot_get_device("Tsensor");
   
  WbDeviceTag motorTp = wb_robot_get_device("Tpmotor");
  WbDeviceTag sensorTp = wb_robot_get_device("Tpsensor");
  
  WbDeviceTag gps =  wb_robot_get_device("gps");
  WbDeviceTag imu = wb_robot_get_device("imu");
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  
  wb_gps_enable(gps, TIME_STEP);
  wb_gyro_enable(gyro, TIME_STEP);
  wb_inertial_unit_enable(imu, TIME_STEP);
  wb_position_sensor_enable(sensorT, TIME_STEP);
  wb_position_sensor_enable(sensorTp, TIME_STEP);


  while (wb_robot_step(TIME_STEP) != -1) {
    x = wb_gps_get_values(gps)[0];
    xdot = (x - lastx) / TIME_STEP * 1000;
    lastx = x;
    fai = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    faidot = wb_gyro_get_values(gyro)[2];
    alpha = -wb_position_sensor_get_value(sensorTp);
    sita = alpha - fai;
    sitadot = (sita - lastsita) / TIME_STEP * 1000;
    lastsita = sita;
  
    ADRC();
  
    wb_motor_set_torque(motorT, -Twout);
    wb_motor_set_torque(motorTp, Tpout);
  };
  wb_robot_cleanup();
  return 0;
}

void LQR() {
// LQR 参数
  float K11 = -5.12622752079414;
  float K12 = -5.77263861915418;
  float K13 = -44.8997809364449;
  float K14 = -1.98850613473999;
  float K15 = 60.7131746019978;
  float K16 = 1.66574663553171;
  float K21 = 17.1722789873743;
  float K22 = 18.8626500693408;
  float K23 = 112.266268814191;
  float K24 = 5.84166267471078;
  float K25 = 72.4958048371721;
  float K26 = -0.0164149717246250;
	Twout = -(K11 * (xexpect - x) + (xdotexpect - xdot) * K12 + (sitaexpect - sita) * K13 + (sitadotexpect - sitadot) * K14 + (faiexpect - fai) * K15 + (faidotexpect - faidot) * K16);
	Tpout = -(K21 * (xexpect - x) + (xdotexpect - xdot) * K22 + (sitaexpect - sita) * K23 + (sitadotexpect - sitadot) * K24 + (faiexpect - fai) * K25 + (faidotexpect - faidot) * K26);
}

void Hinfinty() {
// Hinfinty 参数 gamma = 3.5
  float K11 = 54.5737831582879;
  float K12 = 55.4282672974688;
  float K13 = 623.206774663414;
  float K14 = 68.7197250073667;
  float K15 = 25.2673980392879;
  float K16 = 22.4960823753974;
  float K21 = 52.3074541233526;
  float K22 = 53.3698578559489;
  float K23 = 620.695568279421;
  float K24 = 68.7015176262808;
  float K25 = -4.18325842769431;
  float K26 = -4.24427107738475;
	Twout = -(K11 * (xexpect - x) + (xdotexpect - xdot) * K12 + (sitaexpect - sita) * K13 + (sitadotexpect - sitadot) * K14 + (faiexpect - fai) * K15 + (faidotexpect - faidot) * K16);
	Tpout = -(K21 * (xexpect - x) + (xdotexpect - xdot) * K22 + (sitaexpect - sita) * K23 + (sitadotexpect - sitadot) * K24 + (faiexpect - fai) * K25 + (faidotexpect - faidot) * K26);
}
 
void ADRC() {
  LTD(xexpect, sitaexpect, faiexpect);
  LESO(x, sita, fai, Twout, Tpout);
  LSEFOutput out = LSEF();
  Twout = out.Tw;
  Tpout = -out.Tb;
}


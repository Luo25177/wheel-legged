#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/position_sensor.h>

#define TIME_STEP 10

#define K11 -26.9619
#define K12 -4.9233
#define K13 -13.5970
#define K14 -18.6010
#define K15 2.7496
#define K16 0.3613
#define K21 14.1909
#define K22 2.6502 
#define K23 7.7772
#define K24 10.6004
#define K25 19.2291
#define K26 1.9076

int main(int argc, char **argv) {
  wb_robot_init();
  
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

  double Tout;
  double Tpout;

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
    Tout = K11 * (sitaexpect - sita) + (sitadotexpect - sitadot) * K12 + (xexpect - x) * K13 + (xdotexpect - xdot) * K14 + (faiexpect - fai) * K15 + (faidotexpect - faidot) * K16;
    Tpout = K21 * (sitaexpect - sita) + (sitadotexpect - sitadot) * K22 + (xexpect - x) * K23 + (xdotexpect - xdot) * K24 + (faiexpect - fai) * K25 + (faidotexpect - faidot) * K26;

    wb_motor_set_torque(motorT, Tout);
    wb_motor_set_torque(motorTp, -Tpout);
  };
  wb_robot_cleanup();
  return 0;
}
 

#include "cybergear.h"

#define P_MIN  -12.5f
#define P_MAX  12.5f
#define V_MIN  -30.0f
#define V_MAX  30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN  -12.0f
#define T_MAX  12.0f

int FloatToUint(float _x, float _min, float _max, int _bits) {
  float span   = _max - _min;
  float offset = _min;
  if (_x > _max)
    _x = _max;
  else if (_x < _min)
    _x = _min;
  return (int) ((_x - offset) * ((float) ((1 << _bits) - 1)) / span);
}

// TODO:
void CyberGearInit(CyberGear* motor, u8 id) {
}

void CyberGearStatueControl(u8 controlword, u8 id) {
}

void CyberGearreceiveHandle(CyberGear* motor, CanRxMsg msg) {
}

void CyberGearCommunicate(CyberGear* motor, float _pos, float _speed, float _torque, float _kp, float _kd) {
}

void CyberGearRun(CyberGear* motor) {
}

void CyberGearMonitor(CyberGear* motor) {
}

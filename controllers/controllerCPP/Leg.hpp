#pragma once

#include "Joint.hpp"
#include "Pid.hpp"

class Leg {
  Joint* jointF;
  Joint* jointB;
  Wheel* wheel;

  float angle2, angle3;
  float angle1set, angle4set;
  float angle0;

  float xa, ya;
  float xb, yb;
  float xc, yc;
  float xd, yd;
  float xe, ye;

  public:
  float      angle1;
  float      angle4;
  DataStruct theta;
  DataStruct L0;
  DataStruct dis;
  float      Fnow;
  float      Tbnow;
  float      Twnow;
  float      force;
  float      Fset;
  float      Tbset;
  float      Twset;
  PID        L0PID;

  explicit Leg(const LEGDIR& _dir);
  ~Leg();
  void update();
  void zjie(const float& _pitch);
  void zjie(const float& _pitch, const float& _angle1, const float _angle4);
  void njie(const float& _xc, const float& _yc);
  void run();
  void VMC();
  void INVMC();
};

extern PIDParam legPIDParam;

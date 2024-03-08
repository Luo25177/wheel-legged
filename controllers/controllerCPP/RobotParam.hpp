#pragma once

#include "MyMath.hpp"

#include <Eigen>

#define l1     0.20f
#define l2     0.40f
#define l3     0.40f
#define l4     0.20f
#define l5     0.16f
#define WHEELR 0.075f

#define MASSL1       0.053803f
#define MASSL2       0.080482f
#define MASSL3       0.080482f
#define MASSL4       0.053803f
#define MASSBODY     12.09f
#define HALFMASSBODY 6.045f
#define MASSLEG      0.26857f
#define MASSWHEEL    0.70686f
#define GRAVITY      9.805f

#define FFEEDFORWARD -64.59885f

#define FORCETHRESHOLD  20.f
#define MAXROBOTSPEED   1.f
#define MINROBOTLEGLEN  0.26f
#define MAXROBOTLEGLEN  0.55f
#define MAXROBOTLEGDIFF 0.1f
#define MAXROBOTROLL    1.f
#define MAXROBOTSPLIT   1

#define InitAngle1 5.0 / 6.0 * PI
#define InitAngle4 1.0 / 6.0 * PI

#define LTDBANDWITH 0.5f

#define DT       0.005f
#define TIMESTEP 5

enum LEGDIR {
  LEFT,
  RIGHT
};

enum RobotControlMode {
  SPLIT,
  WBC,
  MPC
};

enum JumpPhase {
  ON,
  KICK,
  SHRINK,
  BUFFER,
  OFF,
};

struct DataStruct {
  float now;
  float dot;
  float ddot;

  float last;
  float lastdot;
};


#pragma once

#include "robotparam.h"
#include <math.h>

typedef struct {
  bool finish;
  float xset;
  float speed;
  float x;
  float startSpeed;
  float uniformSpeed;
  float endSpeed;
  float aimx;
  float speedUpDis;
  float speedDownDis;
} LineTraj;

void LineTrajSpeedUp(LineTraj *linetraj);
void LineTrajSpeedDown(LineTraj *linetraj);
void LineTrajRun(LineTraj *linetraj, float xnow);
void LineTrajInit(LineTraj *linetraj, float startspeed, float uniformspeed, float endspeed, float aimx, float speedUpDis, float speedDownDis);

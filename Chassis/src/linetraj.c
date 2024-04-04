#include "linetraj.h"

void LineTrajSpeedUp(LineTraj* linetraj) {
  if (linetraj->speed < linetraj->uniformSpeed)
    linetraj->speed = linetraj->startSpeed + (linetraj->uniformSpeed - linetraj->startSpeed) * sqrtf(linetraj->x / linetraj->speedUpDis);
}

void LineTrajSpeedDown(LineTraj* linetraj) {
  float surplusDis = linetraj->aimx - linetraj->x;
  if (surplusDis <= linetraj->speedDownDis)
    linetraj->speed = linetraj->endSpeed + (linetraj->uniformSpeed - linetraj->endSpeed) * sqrtf(surplusDis / linetraj->speedDownDis);
}

void LineTrajRun(LineTraj* linetraj, float xnow) {
  if (linetraj->finish) {
    linetraj->xset     = linetraj->aimx;
    linetraj->speed = 0;
    return;
  }
  linetraj->x = xnow;
  if (linetraj->x < 0)
    linetraj->x = 0;
  if (xnow < linetraj->speedUpDis)
    LineTrajSpeedUp(linetraj);
  else if (xnow > linetraj->aimx) {
    linetraj->finish = true;
    linetraj->x      = linetraj->aimx;
    linetraj->speed  = 0;
  } else if (xnow > linetraj->aimx - linetraj->speedDownDis)
    LineTrajSpeedDown(linetraj);
  else
    linetraj->speed = linetraj->uniformSpeed;
  linetraj->xset += linetraj->speed * 0.005f;
}

void LineTrajInit(LineTraj* linetraj, float startspeed, float uniformspeed, float endspeed, float aimx, float speedUpDis, float speedDownDis) {
  linetraj->finish       = false;
  linetraj->startSpeed   = startspeed;
  linetraj->uniformSpeed = uniformspeed;
  linetraj->endSpeed     = endspeed;
  linetraj->aimx         = aimx;
  linetraj->speedUpDis   = speedUpDis;
  linetraj->speedDownDis = speedDownDis;
  linetraj->x            = 0;
  linetraj->xset         = 0;
  linetraj->speed        = 0;
}
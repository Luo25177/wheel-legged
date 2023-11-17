#include "robotparam.h"

float Kcoeff[12][4] = {0};

void inputInit(Input* input) {
  input->theta = 0;
  input->thetadot = 0;
  input->x = 0;
  input->v = 0;
  input->pitch = 0;
  input->pitchdot = 0;
}

void outputInit(Output* output) {
  output->Tp = 0;
  output->Twheel = 0;
}

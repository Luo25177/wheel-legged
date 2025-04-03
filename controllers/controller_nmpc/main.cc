#include "car.hh"
#include <cstdio>

int main(int argc, char **argv) {
  Car *robot = new Car();
    FILE *fp;
    fopen_s(&fp, "../../data/Data.xlsx", "w");
  while (robot->step(time_step) != -1) {
    robot->update();
    robot->data_show(fp);
    robot->run_nmpc();
  };
  delete robot;
  fclose(fp);
  return 0;
}


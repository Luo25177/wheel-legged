#include <webots/robot.h>
#include "car.h"


int main(int argc, char **argv) {
  wb_robot_init();
  robotInit();

  while (wb_robot_step(timestep) != -1) {
    updateState();
    robotRun();
  };

  wb_robot_cleanup();

  return 0;
}

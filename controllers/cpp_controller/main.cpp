#include "luo_robot.hpp"
#include "param.hpp"


#include <cstdio>

int main() {
  while (1) {
    robot_typedef* robot = new robot_typedef();
    FILE *fp;
    fopen_s(&fp, "../../data/Data.xlsx", "w");

    while (robot->step(time_step) != -1) {
      robot->update();
      fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\n", robot->leg_l->t_motor1,
              robot->leg_l->t_motor2, robot->leg_l->t_motor3,
              robot->leg_r->t_motor1, robot->leg_r->t_motor2,
              robot->leg_r->t_motor3);
      robot->run();
    }
    delete robot;
    return 0;
  }
}

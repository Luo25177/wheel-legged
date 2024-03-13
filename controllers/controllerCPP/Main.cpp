#include "Main.hpp"

int main(int argc, char** argv) {
  Car* car = new Car();
  while (car->step(TIMESTEP) != -1) {
    car->update();
    car->SplitLQRControl();
  }
  delete car;
  return 0;
}

#include "car.h"

#include <webots/keyboard.h>
#include <webots/robot.h>

int main(int argc, char** argv) {
	wb_robot_init();
	robotInit();
	FILE* fp = fopen("../../data/Data.xlsx", "w");
	while (wb_robot_step(timestep) != -1) {
		updateState();
		robotRun();
		fprintf(fp, "%f\t%f\n", car.yesense.z, car.force);
		int key = wb_keyboard_get_key();
		switch (key) {
			case ' ':
				car.mode = ROBOTJUMP;
				break;
			default:
				break;
		}
	};

	wb_robot_cleanup();

	return 0;
}

#include "car.h"

#include <webots/keyboard.h>
#include <webots/robot.h>

int main(int argc, char** argv) {
	wb_robot_init();
	robotInit();
	FILE* fp = fopen("../../data/Data.xlsx", "w");
	wb_keyboard_enable(timestep);
	while (wb_robot_step(timestep) != -1) {
		updateState();
		car.mode = ROBOTWBC;
		robotRun();
		fprintf(fp, "%f\t%f\t%f\t%F\n", car.legVir.X.theta, car.legVir.X.pitch, car.legVir.X.v, car.legVir.Xd.v);
		int new_key = wb_keyboard_get_key();
		while (new_key > 0) {
			switch (new_key) {
				case ' ':
					car.mode = ROBOTJUMP;
					break;
				case WB_KEYBOARD_UP:
					vd = 2.5;
					break;
				case WB_KEYBOARD_DOWN:
					vd = -2.5;
					break;
				case WB_KEYBOARD_LEFT:
					car.yawpid.target = 1.8;
					psid						= 1.8;
					break;
				case WB_KEYBOARD_RIGHT:
					car.yawpid.target = -1.8;
					psid						= -1.8;
					break;
				case 'S':
					car.legL.L0pid.target -= 0.001;
					car.legR.L0pid.target -= 0.001;
					limitIn2Range(float)(&car.legL.L0pid.target, 0.25, 0.55);
					limitIn2Range(float)(&car.legR.L0pid.target, 0.25, 0.55);
					break;
				case 'W':
					car.legL.L0pid.target += 0.001;
					car.legR.L0pid.target += 0.001;
					limitIn2Range(float)(&car.legL.L0pid.target, 0.25, 0.55);
					limitIn2Range(float)(&car.legR.L0pid.target, 0.25, 0.55);
					break;
				default:
					break;
			}
			new_key = wb_keyboard_get_key();
		}
	};

	wb_robot_cleanup();

	return 0;
}

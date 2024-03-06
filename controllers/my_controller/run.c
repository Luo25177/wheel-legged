#include "car.h"

#include <webots/keyboard.h>
#include <webots/robot.h>

int main(int argc, char** argv) {
	wb_robot_init();
	robotInit();
	FILE* fp = fopen("../../data/Data.xlsx", "w");
	car.mode = ROBOTNORMAL;
	wb_keyboard_enable(timestep);
	while (wb_robot_step(timestep) != -1) {
		updateState();
		robotRun();
		//fprintf(fp, "%f\t%f\t%f\t%F\n", car.legVir.X.theta, car.legVir.X.pitch, car.legVir.X.v, car.legVir.Xd.v);
		fprintf(fp, "%f\n", car.yesense.z);
		int new_key = wb_keyboard_get_key();
		while (new_key > 0) {
			switch (new_key) {
				case ' ':
					if (!jumpFlag) 
						car.jumpPhase = ON;
					jumpFlag = true;
					break;
				case WB_KEYBOARD_UP:
					vd = 4;
					break;
				case WB_KEYBOARD_DOWN:
					vd = -4;
					break;
				case WB_KEYBOARD_LEFT:
					car.yawpid.target	 = 1.8;
					psid							+= 0.012;
					break;
				case WB_KEYBOARD_RIGHT:
					car.yawpid.target	 = -1.8;
					psid							+= -0.012;
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

#include "robotparam.h"

void inputInit(Input* input) {
	input->theta		= 0;
	input->thetadot = 0;
	input->x				= 0;
	input->v				= 0;
	input->pitch		= 0;
	input->pitchdot = 0;
}

void outputInit(Output* output) {
	output->Tp		 = 0;
	output->Twheel = 0;
}

// TODO:
vector2f jumpPoints[4] = {

};
float Kcoeff[12][4] = { 17.01888975,	-0.903154769, -31.57238975, -41.9788821,	85.78235496, -49.52948422, -40.63369617, 40.35634291,
												42.03139839,	-53.61799535, 13.57488613,	-13.64542004, 18.23634353, -7.239930703, -12.5439447,	 11.26342392,
												-2.884425269, 6.643548023,	-5.128265678, -8.547027497, 9.049735285, 4.590669416,	 -17.93157685, 10.55885787,
												10.96680271,	-13.28880131, 4.268582383,	-11.41664036, 5.762044557, 11.65149503,	 -23.13892639, 12.2324679,
												46.51992207,	-6.127866058, -50.17355563, 33.52934316,	30.79277867, -76.25101817, 61.17595912,	 123.0752698,
												-4.773544463, 9.723701106,	-7.737713765, 2.793511935,	3.104944241, -6.038839679, 4.125493328,	 4.549881246 };
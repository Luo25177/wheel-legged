#pragma once
#include "leg.h"
#include "pid.h"
#include "robotparam.h"
#include "yesense.h"

typedef struct {
	bool flyflag;

	Leg			legL;
	Leg			legR;
	Leg			legVir;
	Yesense yesense;

	PID yawpid;	
	PID rollpid;
	PID splitpid;

	RobotRunMode mode;

	float L0Set;
	float xSet;
	float vSet;

	float force;
} Car;

extern Car car;
extern float vd;

void			 robotInit();
void			 updateState();
void			 balanceMode();
void			 jumpMode();
void			 haltMode();
void			 flyCheck();
void			 robotRun();

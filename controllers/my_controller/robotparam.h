#pragma once

//----
// @file robotparam.h
// @author mask <beloved25177@126.com>
// @brief
// @version 1.0
// @date 2023-11-13
//
// @copyright Copyright (c) 2023
//
//----
#pragma once
#include "stdbool.h"

#define l1		 0.20f
#define l2		 0.40f
#define l3		 0.40f
#define l4		 0.20f
#define l5		 0.16f
#define WHEELR 0.075f

#define MASSL1			 0.053803f
#define MASSL2			 0.080482f
#define MASSL3			 0.080482f
#define MASSL4			 0.053803f
#define MASSBODY		 12.09f
#define HALFMASSBODY 6.045f
#define MASSLEG			 0.26857f
#define MASSWHEEL		 0.70686f
#define GRAVITY			 9.805f

#define FFEEDFORWARD -64.59885f

#define FORCETHRESHOLD	-20.f
#define MAXROBOTSPEED		1.f
#define MINROBOTLEGLEN	0.25f;
#define MAXROBOTLEGLEN	0.55f;
#define MAXROBOTLEGDIFF 0.1f;
#define MAXROBOTROLL		1.f;
#define MAXROBOTSPLIT		1;
#define LEGLEFT					(int) 1
#define LEGRIGHT				(int) -1

#define timestep 5
#define dt			 0.005f

#define InitAngle1 5.0 / 6.0 * PI
#define InitAngle4 1.0 / 6.0 * PI

typedef enum {
	ROBOTNORMAL = 0,
	ROBOTJUMP,
	ROBOTHALT,
	ROBOTWBC
} RobotRunMode;

typedef enum {
	ON,
	KICK,
	SHRINK,
	BUFFER,
	OFF
} JumpPhase;

typedef struct
{
	float theta;
	float thetadot;
	float x;
	float v;
	float pitch;
	float pitchdot;
} Input;

typedef struct
{
	float Tp;
	float Twheel;
} Output;

void inputInit(Input* input);
void outputInit(Output* output);

extern float Kcoeff[12][4];
extern float Kcoeff_wbc[40][6];
extern bool	 jumpFlag;


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

	// �����ĸ�PID��������Ϊ��
	PID yawpid;		 // ���ٶȿ���
	PID rollpid;	 // �����ǿ���
	PID splitpid;	 // ˫���������

	RobotRunMode mode;

	float L0Set;	// �趨�ȳ���Ҳ���ǵ�ǰ�����ȵ�ƽ���ȳ�
	float xSet;
	float vSet;

	float force;
} Car;

extern Car car;
void			 robotInit();
void			 updateState();
void			 balanceMode();
void			 jumpMode();
void			 haltMode();
void			 flyCheck();
void			 robotRun();

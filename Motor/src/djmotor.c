#include "djmotor.h"

#define M3508MAXPULSE							8192
#define M3508PULSETHRESHOLD				4096
#define M3508MAXCURRENT						14745
#define M3508MAXSPEED							8550.f
#define M3508ZEROSPEED						1000.f	// 寻零或者失能的最大速度
#define M3508RATIO								19.2f
#define M3508ANGLETOPULSE					436.90667f			 // 角度转为编码数 deg->pulse
#define M3508TTOI									2730.6667f			 // 扭矩转电流
#define M3508ITOT									0.000366210938f	 // 电流转扭矩
#define M3508FINISHPULSETHRESHOLD 60.f						 // 电机到位判定的阈值

void DJmotorInit(DJmotor* motor, u8 id) {
	for (int i = 0; i < 2; ++i) {
		motor[i].id							 = id++;
		motor[i].n							 = 0;
		motor[i].setZero				 = false;
		motor[i].monitor.timeOut = false;
		motor[i].monitor.mode		 = TORQUE;
		motor[i].monitor.enable	 = false;
		motor[i].speedPid				 = (PID*) malloc(sizeof(PID));
		motor[i].pulsePid				 = (PID*) malloc(sizeof(PID));
		motor[i].set.current		 = 0;
		motor[i].set.angleDeg		 = 0;
		motor[i].set.angleRad		 = 0;
		motor[i].set.velocity		 = 0;
		motor[i].set.torque			 = 0;
		// 增量式PID
		pidInit(motor[i].speedPid, 8, 0.25, 0, 0, 0, PIDINC);
		pidInit(motor[i].pulsePid, 1.3, 0.2, 0, M3508MAXSPEED, 0, PIDINC);
	}
}

void DJmotorreceiveHandle(DJmotor* motor, CanRxMsg msg) {
	int id											 = msg.StdId - 0x201;
	motor[id].monitor.timeOutCnt = 0;
	if (motor[id].monitor.timeOut)
		motor[id].monitor.timeOut = false;
	BU8ToVS16(msg.Data, &motor[id].pulseRead);
	if (!motor[id].setZero) {
		motor[id].setZero				= true;
		motor[id].lockPulse			= motor[id].pulseRead;
		motor[id].lastPulseRead = motor[id].pulseRead;
	}
	if ((motor[id].pulseRead - motor[id].lastPulseRead) > M3508PULSETHRESHOLD)
		--motor[id].n;
	else if ((motor[id].lastPulseRead - motor[id].pulseRead) > M3508PULSETHRESHOLD)
		++motor[id].n;
	motor[id].lastPulseRead = motor[id].pulseRead;
	BU8ToVS16(msg.Data + 2, &motor[id].real.velocity);
	BU8ToVS16(msg.Data + 4, &motor[id].real.current);
	BU8ToVS16(msg.Data + 6, &motor[id].temperature);
	motor[id].pulseAccumulate = motor[id].n * M3508MAXPULSE + motor[id].pulseRead - motor[id].lockPulse;
	motor[id].real.angleDeg		= motor[id].pulseAccumulate / M3508ANGLETOPULSE;
	motor[id].real.torque			= (float) (motor[id].real.current * M3508ITOT);
	motor[id].real.angleRad		= motor[id].real.angleDeg * DegToRad;
}

void DJmotorCommunicate(DJmotor* motor, u32 stdid) {
	CanTxMsg txmsg;
	txmsg.IDE		= CAN_Id_Standard;
	txmsg.RTR		= CAN_RTR_DATA;
	txmsg.StdId = stdid;
	txmsg.DLC		= 0x08;

	for (int i = 0; i < 2; ++i) BS16ToU8(&motor[i].set.current, &txmsg.Data[i << 1]);
	can2Txmsg->push(can2Txmsg, txmsg);
}

void DJmotorRun(DJmotor* motor) {
	for (int i = 0; i < 2; ++i) {
		if (!motor[i].monitor.enable) {
			motor[i].set.current = 0;
			continue;
		}
		switch (motor[i].monitor.mode) {
			case HALT:
				motor[i].set.current = 0;
				break;
			case POSITION:
				motor[i].pulsePid->target = motor[i].set.angleDeg * M3508ANGLETOPULSE;
				motor[i].set.velocity			= motor[i].pulsePid->compute(motor[i].pulsePid, (float) motor[i].pulseAccumulate);
			case SPEED:
				motor[i].speedPid->target	 = motor[i].set.velocity;
				motor[i].set.current			+= motor[i].speedPid->compute(motor[i].speedPid, (float) motor[i].real.velocity);
				limitInRange(s16)(&motor[i].set.current, M3508MAXCURRENT);
				break;
			case TORQUE:
				motor[i].set.current = (s16) (motor[i].set.torque * M3508TTOI);
				limitInRange(s16)(&motor[i].set.current, M3508MAXCURRENT);
				break;
			default:
				motor[i].set.current = 0;
				break;
		}
	}
	DJmotorCommunicate(motor, (u32) 0x200);
}

void DJmotorMonitor(DJmotor* motor) {
	for (int i = 0; i < 2; ++i) {
		++motor[i].monitor.timeOutCnt;
		if (motor[i].monitor.timeOutCnt >= 10) {
			motor[i].monitor.timeOut = true;
		}
	}
}

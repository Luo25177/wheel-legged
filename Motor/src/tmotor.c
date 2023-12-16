#include "tmotor.h"

// 电机极限参数（见电机说明）
#define P_MIN	 -95.5f	 // 位置极限（rad）
#define P_MAX	 95.5f
#define V_MIN	 -50.0f	 // 速度极限（Rad/s）
#define V_MAX	 50.0f
#define KP_MIN 0.0f	 // KP极限（N-m/rad）
#define KP_MAX 500.0f
#define KD_MIN 0.0f	 // KD极限（N-m/rad/s）
#define KD_MAX 5.0f
#define T_MIN	 -12.0f	 // 扭矩极限（Nm）
#define T_MAX	 12.0f
#define TRATIO 7.643f
//----
// @brief 初始化
//
// @param motor
// @param id
//----
void TmotorInit(Tmotor* motor, u8 id) {
	for (int i = 0; i < 4; ++i) {
		// 电机初始参参数
		motor[i].id									 = id;
		motor[i].kp									 = 80;
		motor[i].kd									 = 0.66;
		motor[i].set.torque					 = 0;
		motor[i].set.velocity				 = 0;
		motor[i].set.angleDeg				 = 0;
		motor[i].set.angleRad				 = 0;
		motor[i].set.current				 = 0;

		motor[i].monitor.mode				 = TORQUE;
		motor[i].monitor.enable			 = false;
		motor[i].monitor.timeOut		 = false;
		motor[i].monitor.stuckCnt		 = 0;
		motor[i].monitor.timeOutCnt	 = 0;
		motor[i].monitor.stuckRealse = true;
		// 进入控制模式
		TmotorStatueControl(TENTERCONTROL, id++);
	}
	for (int i = 5; i <= 8; ++i) {
		TmotorStatueControl(TENTERCONTROL, i);
	}
}

//----
// @brief 电机状态控制
//
// @param controlword
// @param id
//----
void TmotorStatueControl(u8 controlword, u8 id) {
	CanTxMsg txmsg;
	txmsg.StdId		= 0x00 + id;
	txmsg.RTR			= CAN_RTR_DATA;
	txmsg.IDE			= CAN_Id_Standard;
	txmsg.DLC			= 0x08;
	txmsg.Data[0] = 0xff;
	txmsg.Data[1] = 0xff;
	txmsg.Data[2] = 0xff;
	txmsg.Data[3] = 0xff;
	txmsg.Data[4] = 0xff;
	txmsg.Data[5] = 0xff;
	txmsg.Data[6] = 0xff;

	switch (controlword) {
		case TENTERCONTROL:
			txmsg.Data[7] = 0xfc;
			break;
		case TEXITCONTROL:
			txmsg.Data[7] = 0xfd;
			break;
		case TSETZERO:
			txmsg.Data[7] = 0xfe;
			break;
	}
	can1Txmsg->push(can1Txmsg, txmsg);
}

//----
// @brief 数据类型转换，主要是由于 Tmotor 这种该死的数据格式没有对齐
//----
static u16 float2uint(float x, float x_min, float x_max, u8 bits)	 // 浮点型化为整型
{
	float span	 = x_max - x_min;
	float offset = x_min;
	return (u16) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}
static float uint2float(int x_int, float x_min, float x_max, int bits)	// 整型化为浮点型
{
	float span	 = x_max - x_min;
	float offset = x_min;
	return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

//----
// @brief 电机运动控制
//
// @param motor
// @param _pos
// @param _speed
// @param _torque
// @param _kp
// @param _kd
//----
void TmotorCommunicate(Tmotor* motor, float _pos, float _speed, float _torque, float _kp, float _kd) {
	_pos	 *= DegToRad * TRATIO;
	_speed *= DegToRad * TRATIO;

	u16 p		= float2uint(_pos, P_MIN, P_MAX, 16);
	u16 v		= float2uint(_speed, V_MIN, V_MAX, 12);
	u16 t		= float2uint(_torque, T_MIN, T_MAX, 12);
	u16 kp	= float2uint(_kp, KP_MIN, KP_MIN, 12);
	u16 kd	= float2uint(_kd, KD_MIN, KD_MAX, 12);

	CanTxMsg txmsg;
	txmsg.StdId		= 0x00 + motor->id;
	txmsg.IDE			= CAN_Id_Standard;
	txmsg.RTR			= CAN_RTR_Data;
	txmsg.DLC			= 0x08;
	txmsg.Data[0] = (p >> 8) & 0xff;
	txmsg.Data[1] = p & 0xff;
	txmsg.Data[2] = v >> 4;
	txmsg.Data[3] = ((v & 0xf) << 4) | (kp >> 8);
	txmsg.Data[4] = kp & 0xff;
	txmsg.Data[5] = kd >> 4;
	txmsg.Data[6] = ((kd & 0xf) << 4) | (t >> 8);
	txmsg.Data[7] = t & 0xff;

	can1Txmsg->push(can1Txmsg, txmsg);
}

//----
// @brief 电机接收消息处理
//
// @param motor
// @param msg
//----
void TmotorreceiveHandle(Tmotor* motor, CanRxMsg msg) {
	u8	id											 = msg.Data[0] - 1;
	u16 p												 = (msg.Data[1] << 8) | msg.Data[2];				 // 电机位置
	u16 v												 = (msg.Data[3] << 4) | (msg.Data[4] >> 4);	 // 电机速度
	u16 t												 = (msg.Data[4] & 0x0f) << 8 | msg.Data[5];	 // 电机扭矩

	motor[id].monitor.timeOutCnt = 0;
	if (motor[id].monitor.timeOut)
		motor[id].monitor.timeOut = false;
	motor[id].real.angleDeg = uint2float(p, P_MIN, P_MAX, 16) * RadToDeg / TRATIO;
	motor[id].real.velocity = uint2float(v, P_MIN, P_MAX, 12) * RadToDeg / TRATIO;
	motor[id].real.torque		= uint2float(t, T_MIN, T_MAX, 12);
	if (!motor[id].init) {
		motor[id].initReadAngle = motor[id].real.angleDeg;
		motor[id].init					= true;
	}
	motor[id].real.angleDeg -= motor[id].initReadAngle;
	motor[id].real.angleRad	 = motor[id].real.angleDeg * DegToRad;
}

void TmotorEnable(Tmotor* motor, u8 controlword) {
	if (controlword & 0x01) {
		motor->monitor.enable = true;
	} else
		motor->monitor.enable = false;
}

void TmotorRun(Tmotor* motor) {
	for (int i = 0; i < 4; ++i) {
		if (!motor[i].monitor.enable) {
			motor[i].set.angleDeg = motor[i].real.angleDeg;
			continue;
		}
		switch (motor[i].monitor.mode) {
			case HALT:
				TmotorCommunicate(&motor[i], 0, 0, 0, 0, 0);
				break;
			case POSITION:
				TmotorCommunicate(&motor[i], motor[i].set.angleDeg + motor[i].initReadAngle, 0, motor[i].kp, motor[i].kd, 0);
				break;
			case SPEED:
				TmotorCommunicate(&motor[i], 0, motor[i].set.velocity, 0, motor[i].kp, 0);
				break;
			case TORQUE:
				TmotorCommunicate(&motor[i], 0, 0, motor[i].set.torque, 0, 0);
				break;
			default:
				TmotorCommunicate(&motor[i], 0, 0, 0, 0, 0);
				break;
		}
	}
}

void TmotorMonitor(Tmotor* motor) {
	for (int i = 0; i < 4; ++i) {
		++motor[i].monitor.timeOutCnt;
		if (motor[i].monitor.timeOutCnt >= 10)
			motor[i].monitor.timeOut = true;
	}
}

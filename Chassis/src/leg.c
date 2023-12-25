#include "leg.h"

//----
// @brief 初始化
//
// @param leg
// @param dir 腿的方向
//----
void legInit(Leg* leg, int dir, DJmotor* wheel, Tmotor* front, Tmotor* behind) {
	leg->dir = dir;

	pidInit(&leg->L0pid, 500, 2.2, 1600, 0, 1000, PIDPOS);

	datastructInit(&leg->dis, 0, 0, 0, 0);
	datastructInit(&leg->L0, 0, 0, 0, 0);
	datastructInit(&leg->angle0, 0, 0, 0, 0);

	leg->wheel								= wheel;
	leg->front								= front;
	leg->behind								= behind;

	leg->Fset									= 0;
	leg->Tpset								= 0;
	leg->TFset								= 0;
	leg->TBset								= 0;
	leg->TWheelset						= 0;
	leg->timer								= 0;
	leg->normalforce					= 0;
	// 电机绑定
	leg->front->initSetAngle	= FrontAngleInit;
	leg->behind->initSetAngle = BehindAngleInit;

	inputInit(&leg->X);
	inputInit(&leg->Xd);
	outputInit(&leg->U);
}

//----
// @brief 腿部状态量更新
//
// @param leg
//----
void legUpdate(Leg* leg) {
	leg->angle1		 = (float) (leg->front->initSetAngle + (leg->front->real.angleRad * leg->dir));
	leg->angle4		 = (float) (leg->behind->initSetAngle + (leg->behind->real.angleRad * leg->dir));
	leg->dis.now	 = (float) leg->wheel->real.angleRad * WHEELR * leg->dir;
	leg->dis.dot	 = (float) leg->wheel->real.velocity * M3508RPMTORAD * WHEELR * leg->dir;
	leg->TFnow		 = (float) leg->front->real.torque * leg->dir;
	leg->TBnow		 = (float) leg->behind->real.torque * leg->dir;
	leg->TWheelnow = (float) leg->wheel->real.torque * leg->dir;
	INVMC(leg);
}

//----
// @brief 计算L0和angle0的原值和导数值
//
// @param leg
// @param pitch 俯仰角 用于坐标系的转换
//----
void Zjie(Leg* leg, float pitch) {
	leg->xb			= l1 * cos(leg->angle1) - l5 / 2;
	leg->yb			= l1 * sin(leg->angle1);
	leg->xd			= l5 / 2 + l4 * cos(leg->angle4);
	leg->yd			= l4 * sin(leg->angle4);
	float lbd		= sqrt(pow((leg->xd - leg->xb), 2) + pow((leg->yd - leg->yb), 2));
	float A0		= 2 * l2 * (leg->xd - leg->xb);
	float B0		= 2 * l2 * (leg->yd - leg->yb);
	float C0		= pow(l2, 2) + pow(lbd, 2) - pow(l3, 2);
	float D0		= pow(l3, 2) + pow(lbd, 2) - pow(l2, 2);
	leg->angle2 = 2 * atan2((B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(C0, 2))), (A0 + C0));
	leg->angle3 = PI - 2 * atan2((-B0 + sqrt(pow(A0, 2) + pow(B0, 2) - pow(D0, 2))), (A0 + D0));
	leg->xc			= leg->xb + l2 * cos(leg->angle2);
	leg->yc			= leg->yb + l2 * sin(leg->angle2);
	leg->L0.now = sqrt(pow(leg->xc, 2) + pow(leg->yc, 2));

	// 乘以pitch的旋转矩阵
	float cor_XY_then[2][1];
	cor_XY_then[0][0] = cos(pitch) * leg->xc - sin(pitch) * leg->yc;
	cor_XY_then[1][0] = sin(pitch) * leg->xc + cos(pitch) * leg->yc;
	leg->angle0.now		= atan2(cor_XY_then[0][0], cor_XY_then[1][0]);

	float dt					= 1000 / (float) (GlobalTimer - leg->timer);
	leg->timer				= GlobalTimer;

	if (dt > 0) {
		leg->L0.dot					= (leg->L0.now - leg->L0.last) * dt;
		leg->L0.ddot				= (leg->L0.dot - leg->L0.lastdot) * dt;
		leg->L0.last				= leg->L0.now;
		leg->L0.lastdot			= leg->L0.dot;
		leg->angle0.dot			= (leg->angle0.now - leg->angle0.last) * dt;
		leg->angle0.ddot		= (leg->angle0.dot - leg->angle0.lastdot) * dt;
		leg->angle0.last		= leg->angle0.now;
		leg->angle0.lastdot = leg->angle0.dot;
	}
}

//----
// @brief 腿部结构逆解，用于解算出脚端落点的对应的电机角度
//
// @param leg
// @param xc
// @param yc
//----
void Njie(Leg* leg, float xc, float yc) {
	float A, B, C;

	A							 = 2 * (xc + l5 / 2) * l1;
	B							 = 2 * yc * l1;
	C							 = (xc + l5 / 2) * (xc + l5 / 2) + yc * yc + l1 * l1 - l2 * l2;
	leg->angle1set = 2 * atan2((B + sqrt(A * A + B * B - C * C)), (A + C));
	A							 = 2 * l4 * (xc - l5 / 2);
	B							 = 2 * l4 * yc;
	C							 = (xc - l5 / 2) * (xc - l5 / 2) + l4 * l4 + yc * yc - l3 * l3;
	leg->angle4set = 2 * atan2((B - sqrt(A * A + B * B - C * C)), (A + C));
	if (leg->angle4set < 0)
		leg->angle4set += 2 * PI;
}

//----
// @brief VMC 解算，虚拟力->实际电机扭矩
//
// @param leg
//----
void VMC(Leg* leg) {
	float trans[2][2] = { l1 * cos(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) / sin(leg->angle2 - leg->angle3),
												l1 * sin(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) / (leg->L0.now * sin(leg->angle2 - leg->angle3)),
												l4 * cos(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4) / sin(leg->angle2 - leg->angle3),
												l4 * sin(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4) / (leg->L0.now * sin(leg->angle2 - leg->angle3)) };
	leg->TFset				= trans[0][0] * leg->Fset + trans[0][1] * leg->Tpset;
	leg->TBset				= trans[1][0] * leg->Fset + trans[1][1] * leg->Tpset;
}

//----
// @brief 逆向VMC解算 实际电机扭矩->虚拟力
//
// @param leg
//----
void INVMC(Leg* leg) {
	float trans[2][2] = { -(sin(leg->angle0.now + leg->angle2) * sin(leg->angle2 - leg->angle3)) /
													(l1 * cos(leg->angle0.now + leg->angle2) * sin(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) -
													 l1 * cos(leg->angle0.now + leg->angle3) * sin(leg->angle0.now + leg->angle2) * sin(leg->angle1 - leg->angle2)),
												(sin(leg->angle0.now + leg->angle3) * sin(leg->angle2 - leg->angle3)) /
													(l4 * cos(leg->angle0.now + leg->angle2) * sin(leg->angle0.now + leg->angle3) * sin(leg->angle3 - leg->angle4) -
													 l4 * cos(leg->angle0.now + leg->angle3) * sin(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4)),
												(leg->L0.now * cos(leg->angle0.now + leg->angle2) * sin(leg->angle2 - leg->angle3)) /
													(l1 * cos(leg->angle0.now + leg->angle2) * sin(leg->angle0.now + leg->angle3) * sin(leg->angle1 - leg->angle2) -
													 l1 * cos(leg->angle0.now + leg->angle3) * sin(leg->angle0.now + leg->angle2) * sin(leg->angle1 - leg->angle2)),
												-(leg->L0.now * cos(leg->angle0.now + leg->angle3) * sin(leg->angle2 - leg->angle3)) /
													(l4 * cos(leg->angle0.now + leg->angle2) * sin(leg->angle0.now + leg->angle3) * sin(leg->angle3 - leg->angle4) -
													 l4 * cos(leg->angle0.now + leg->angle3) * sin(leg->angle0.now + leg->angle2) * sin(leg->angle3 - leg->angle4)) };

	leg->Fnow					= trans[0][0] * leg->TFnow + trans[0][1] * leg->TBnow;
	leg->Tpnow				= trans[1][0] * leg->TFnow + trans[1][1] * leg->TBnow;
}

#include "car.h"

Car car;

//----
// @brief 初始化
//
//----
void robotInit() {
	yesenseInit(&car.yesense);

	legInit(&car.legVir, LEGLEFT);
	legInit(&car.legR, LEGRIGHT);
	legInit(&car.legL, LEGLEFT);


	car.flyflag	= false;
	car.mode			= ROBOTNORMAL;

	// TODO: 参数暂定 调节
	pidInit(&car.yawpid, 1, 0, 4, 0, 1000, PIDPOS);
	pidInit(&car.rollpid, 100, 0, 1000, 0, 1000, PIDPOS);
	pidInit(&car.splitpid, 100, 0, 300, 0, 1000, PIDPOS);

	car.L0Set							= 0.36;
	car.yawpid.target			= 0;
	car.rollpid.target		= 0;
	car.splitpid.target		= 0;
	car.legL.L0pid.target = car.L0Set;
	car.legR.L0pid.target = car.L0Set;
}

//----
// @brief 状态量更新和一些检查 对控制器的输入量就不在这里做处理了，毕竟这个运行频率有点高
//
//----
void updateState() {
	yesenseUpdata(&car.yesense);
	car.legL.angle1 = 5.0 / 6.0 * PI + wb_position_sensor_get_value(car.legL.frontEncoder);
	car.legL.angle4 = 1.0 / 6.0 * PI - wb_position_sensor_get_value(car.legL.behindEncoder);
	car.legR.angle1 = 5.0 / 6.0 * PI + wb_position_sensor_get_value(car.legR.frontEncoder);
	car.legR.angle4 = 1.0 / 6.0 * PI - wb_position_sensor_get_value(car.legR.behindEncoder);

	legUpdate(&car.legL);
	legUpdate(&car.legR);

	Zjie(&car.legL, car.yesense.pitch.now);
	Zjie(&car.legR, car.yesense.pitch.now);

	car.legVir.angle1 = (car.legL.angle1 + car.legR.angle1) / 2;
	car.legVir.angle4 = (car.legL.angle4 + car.legR.angle4) / 2;
	car.legVir.dis.now = (car.legL.dis.now + car.legR.dis.now) / 2;
	car.legVir.dis.dot = (car.legL.dis.dot + car.legR.dis.dot) / 2;
	Zjie(&car.legVir, car.yesense.pitch.now);

	car.legL.TFnow = car.legL.TFset;
	car.legL.TBnow = -car.legL.TBset;
	car.legL.TWheelnow = car.legL.TWheelset;

	car.legR.TFnow = car.legR.TFset;
	car.legR.TBnow = -car.legR.TBset;
	car.legR.TWheelnow = car.legR.TWheelset;

	flyCheck();
}

//----
// @brief lqr 控制保持平衡
//
//----
void balanceMode() {
	float L03 = pow(car.legVir.L0.now, 3);
	float L02 = pow(car.legVir.L0.now, 2);
	float L01 = pow(car.legVir.L0.now, 1);

	if (car.flyflag) {
		for (int col = 0; col < 6; col++) {
			for (int row = 0; row < 2; row++) {
				int num = (col << 1) + row;
				if (row == 1 && (col == 0 || col == 1))
					car.legVir.K[row][col] = (Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3]);
				else
					car.legVir.K[row][col] = 0;
			}
		}
	} else {
		for (int col = 0; col < 6; col++) {
			for (int row = 0; row < 2; row++) {
				int num										= (col << 1) + row;
				car.legVir.K[row][col] = (Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3]);
			}
		}
	}
	car.legVir.X.theta = car.legVir.angle0.now;
	car.legVir.X.thetadot = car.legVir.angle0.dot;
	car.legVir.X.x = car.legVir.dis.now;
	car.legVir.X.v = car.legVir.dis.dot;
	car.legVir.X.pitch = car.yesense.pitch.now;
	car.legVir.X.pitchdot = car.yesense.pitch.dot;

	static float vd = 0.00;
	car.legVir.Xd.theta		= 0;
	car.legVir.Xd.thetadot = 0;
	car.legVir.Xd.x += vd;// car.legVir.dis.now;
	car.legVir.Xd.v = 0;// car.legVir.dis.dot;
	car.legVir.Xd.pitch	= 0;
	car.legVir.Xd.pitchdot = 0;

	car.legVir.U.Twheel		= car.legVir.K[0][0] * (car.legVir.Xd.theta - car.legVir.X.theta) +
													 car.legVir.K[0][1] * (car.legVir.Xd.thetadot - car.legVir.X.thetadot) +
													 car.legVir.K[0][2] * (car.legVir.Xd.x - car.legVir.X.x) +
													 car.legVir.K[0][3] * (car.legVir.Xd.v - car.legVir.X.v) +
													 car.legVir.K[0][4] * (car.legVir.Xd.pitch - car.legVir.X.pitch) +
													 car.legVir.K[0][5] * (car.legVir.Xd.pitchdot - car.legVir.X.pitchdot);
	car.legVir.U.Tp = car.legVir.K[1][0] * (car.legVir.Xd.theta - car.legVir.X.theta) +
											 car.legVir.K[1][1] * (car.legVir.Xd.thetadot - car.legVir.X.thetadot) +
											 car.legVir.K[1][2] * (car.legVir.Xd.x - car.legVir.X.x) +
											 car.legVir.K[1][3] * (car.legVir.Xd.v - car.legVir.X.v) +
											 car.legVir.K[1][4] * (car.legVir.Xd.pitch - car.legVir.X.pitch) +
											 car.legVir.K[1][5] * (car.legVir.Xd.pitchdot - car.legVir.X.pitchdot);
	car.legL.TWheelset	 = car.legVir.U.Twheel / 2;
	car.legR.TWheelset	 = car.legVir.U.Twheel / 2;

	car.legL.Tpset = car.legVir.U.Tp / 2;
	car.legR.Tpset = car.legVir.U.Tp / 2;

	// 前馈力
	car.legL.Fset = -61.90455385f;
	car.legR.Fset = -61.90455385f;
	// 补偿虚拟力
	float lfCompensate		 = car.legL.L0pid.compute(&car.legL.L0pid, car.legL.L0.now);
	float rfCompensate		 = car.legR.L0pid.compute(&car.legR.L0pid, car.legR.L0.now);
	car.legL.Fset			-= lfCompensate;
	car.legR.Fset			-= rfCompensate;
// 旋转补偿
	car.yawpid.target = 0;
	float yawCompensate		 = car.yawpid.compute(&car.yawpid, car.yesense.yaw.now);
	car.legL.TWheelset -= yawCompensate;
	car.legR.TWheelset += yawCompensate;
	//// 劈腿补偿
	float splitCompensate	 = car.splitpid.compute(&car.splitpid, car.legL.angle0.now - car.legR.angle0.now);
	car.legL.Tpset			+= splitCompensate;
	car.legR.Tpset			-= splitCompensate;
	//// // 翻滚角补偿
	 float rollCompensate	 = car.rollpid.compute(&car.rollpid, car.yesense.roll.now);
	 car.legL.Fset			-= rollCompensate;
	 car.legR.Fset			+= rollCompensate;

	VMC(&car.legL);
	VMC(&car.legR);

	car.legL.TBset *= -1;
	car.legR.TBset *= -1;

	car.legL.TWheelset *= -1;
	car.legR.TWheelset *= -1;

	limitInRange(float) (&car.legL.TFset, 22);
	limitInRange(float) (&car.legL.TBset, 22);
	limitInRange(float) (&car.legL.TWheelset, 10);

	limitInRange(float) (&car.legR.TFset, 22);
	limitInRange(float) (&car.legR.TBset, 22);
	limitInRange(float) (&car.legR.TWheelset, 10);

	static int t = 0;
	//wb_motor_set_position(car.legL.wheel, t);
	//wb_motor_set_position(car.legR.wheel, t--);

	wb_motor_set_torque(car.legL.front, car.legL.TFset);
	wb_motor_set_torque(car.legL.behind, car.legL.TBset);
	wb_motor_set_torque(car.legL.wheel, car.legL.TWheelset);

	wb_motor_set_torque(car.legR.front, car.legR.TFset);
	wb_motor_set_torque(car.legR.behind, car.legR.TBset);
	wb_motor_set_torque(car.legR.wheel, car.legR.TWheelset);
}

//----
// @brief 跳跃 可以先用简单的位控来实现跳跃
//
//----
void jumpMode() {
	// TODO: 待做 再研究一下子
}

//----
// @brief 宕机模式 实际上就是电机全部设置0电流
//
//----
void haltMode() {
}

//----
// @brief 腾空检测
//
//----
void flyCheck() {
	INVMC(&car.legL);
	INVMC(&car.legR);
	float lp								= car.legL.Fnow * cos(car.legL.angle0.now) + car.legL.Tpnow * sin(car.legL.angle0.now) / car.legL.L0.now;
	float rp								= car.legR.Fnow * cos(car.legR.angle0.now) + car.legR.Tpnow * sin(car.legR.angle0.now) / car.legR.L0.now;

	float zmdd							= car.yesense.accelz * cos(car.yesense.pitch.now) - car.yesense.accelx * sin(car.yesense.pitch.now);

	float lzwdd							= zmdd - car.legL.L0.ddot * cos(car.legL.angle0.now) + 2 * car.legL.L0.dot + car.legL.angle0.dot * sin(car.legL.angle0.now) + car.legL.L0.now * car.legL.angle0.ddot * sin(car.legL.angle0.now) + car.legL.L0.now * car.legL.angle0.dot * car.legL.angle0.dot * cos(car.legL.angle0.now);
	float rzwdd							= zmdd - car.legR.L0.ddot * cos(car.legR.angle0.now) + 2 * car.legR.L0.dot + car.legR.angle0.dot * sin(car.legR.angle0.now) + car.legR.L0.now * car.legR.angle0.ddot * sin(car.legR.angle0.now) + car.legR.L0.now * car.legR.angle0.dot * car.legR.angle0.dot * cos(car.legR.angle0.now);

	car.legR.normalforce = -rp + MASSWHEEL * (GRAVITY + rzwdd);
	car.legL.normalforce = -lp + MASSWHEEL * (GRAVITY + lzwdd);

	float force							= (car.legL.normalforce + car.legR.normalforce) / 2;

	if (force < 20)
		car.flyflag = true;
	else
		car.flyflag = false;
}

//----
// @brief 开始运行
//
//----
void robotRun() {
	switch (car.mode) {
		case ROBOTNORMAL:
			balanceMode();
			break;
		case ROBOTJUMP:
			jumpMode();
			break;
		case ROBOTHALT:
			haltMode();
			break;
		default:
			break;
	}
}


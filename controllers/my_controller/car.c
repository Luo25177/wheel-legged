#include "car.h"

Car		car;
float vd = 0;
float psid = 0;

//----
// @brief 初始化
//
//----
void robotInit() {
	yesenseInit(&car.yesense);

	legInit(&car.legVir, LEGLEFT);
	legInit(&car.legR, LEGRIGHT);
	legInit(&car.legL, LEGLEFT);

	car.flyflag = false;
	car.mode		= ROBOTNORMAL;

	// TODO: 参数暂定 调节
	pidInit(&car.yawpid, 3, 0, 0, 0, 0, PIDPOS);
	pidInit(&car.rollpid, 100, 0, 1000, 0, 1000, PIDPOS);
	pidInit(&car.splitpid, 100, 0, 1000, 0, 1000, PIDPOS);

	car.L0Set							= 0.30;
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
	car.legL.angle1 = InitAngle1 + wb_position_sensor_get_value(car.legL.frontEncoder);
	car.legL.angle4 = InitAngle4 - wb_position_sensor_get_value(car.legL.behindEncoder);
	car.legR.angle1 = InitAngle1 + wb_position_sensor_get_value(car.legR.frontEncoder);
	car.legR.angle4 = InitAngle4 - wb_position_sensor_get_value(car.legR.behindEncoder);

	legUpdate(&car.legL);
	legUpdate(&car.legR);

	Zjie(&car.legL, car.yesense.pitch.now);
	Zjie(&car.legR, car.yesense.pitch.now);

	car.legVir.angle1	 = (car.legL.angle1 + car.legR.angle1) / 2;
	car.legVir.angle4	 = (car.legL.angle4 + car.legR.angle4) / 2;
	car.legVir.dis.now = (car.legL.dis.now + car.legR.dis.now) / 2;
	car.legVir.dis.dot = (car.legL.dis.dot + car.legR.dis.dot) / 2;
	Zjie(&car.legVir, car.yesense.pitch.now);

	car.legL.TFnow		 = car.legL.TFset;
	car.legL.TBnow		 = -car.legL.TBset;
	car.legL.TWheelnow = car.legL.TWheelset;

	car.legR.TFnow		 = car.legR.TFset;
	car.legR.TBnow		 = -car.legR.TBset;
	car.legR.TWheelnow = car.legR.TWheelset;

	flyCheck();
}

//----
// @brief lqr 控制保持平衡
//
//----
void balanceMode() {
	float L03 = car.legVir.L0.now * car.legVir.L0.now * car.legVir.L0.now;
	float L02 = car.legVir.L0.now * car.legVir.L0.now;
	float L01 = car.legVir.L0.now;

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
				int num								 = (col << 1) + row;
				car.legVir.K[row][col] = (Kcoeff[num][0] * L03 + Kcoeff[num][1] * L02 + Kcoeff[num][2] * L01 + Kcoeff[num][3]);
			}
		}
	}

	car.legVir.X.theta		 = car.legVir.angle0.now;
	car.legVir.X.thetadot	 = car.legVir.angle0.dot;
	car.legVir.X.x				 = 0;
	car.legVir.X.v				 = car.legVir.dis.dot;
	car.legVir.X.pitch		 = car.yesense.pitch.now;
	car.legVir.X.pitchdot	 = car.yesense.pitch.dot;

	car.legVir.Xd.theta		 = 0;
	car.legVir.Xd.thetadot = 0;
	car.legVir.Xd.x				 = 0;
	car.legVir.Xd.v				 = vd;
	car.legVir.Xd.pitch		 = 0;
	car.legVir.Xd.pitchdot = 0;
	vd										 = 0;

	car.legVir.U.Twheel		 = car.legVir.K[0][0] * (car.legVir.Xd.theta - car.legVir.X.theta) +
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
	car.legL.TWheelset		 = car.legVir.U.Twheel / 2;
	car.legR.TWheelset		 = car.legVir.U.Twheel / 2;

	car.legL.Tpset				 = car.legVir.U.Tp / 2;
	car.legR.Tpset				 = car.legVir.U.Tp / 2;

	// 前馈力
	car.legL.Fset					 = -61.90455385f;
	car.legR.Fset					 = -61.90455385f;
	// 补偿虚拟力
	float lfCompensate		 = car.legL.L0pid.compute(&car.legL.L0pid, car.legL.L0.now);
	float rfCompensate		 = car.legR.L0pid.compute(&car.legR.L0pid, car.legR.L0.now);
	car.legL.Fset					-= lfCompensate;
	car.legR.Fset					-= rfCompensate;
	// 旋转补偿
	float yawCompensate		 = car.yawpid.compute(&car.yawpid, car.yesense.yaw.dot);
	car.yawpid.target			 = 0;
	car.legL.TWheelset		-= yawCompensate;
	car.legR.TWheelset		+= yawCompensate;
	//// 劈腿补偿
	float splitCompensate	 = car.splitpid.compute(&car.splitpid, car.legL.angle0.now - car.legR.angle0.now);
	car.legL.Tpset				+= splitCompensate;
	car.legR.Tpset				-= splitCompensate;
	//// // 翻滚角补偿
	float rollCompensate	 = car.rollpid.compute(&car.rollpid, car.yesense.roll.now);
	car.legL.Fset					-= rollCompensate;
	car.legR.Fset					+= rollCompensate;

	VMC(&car.legL);
	VMC(&car.legR);

	car.legL.TBset *= -1;
	car.legR.TBset *= -1;

	limitInRange(float)(&car.legL.TFset, 22);
	limitInRange(float)(&car.legL.TBset, 22);
	limitInRange(float)(&car.legL.TWheelset, 10);

	limitInRange(float)(&car.legR.TFset, 22);
	limitInRange(float)(&car.legR.TBset, 22);
	limitInRange(float)(&car.legR.TWheelset, 10);

	wb_motor_set_torque(car.legL.front, car.legL.TFset);
	wb_motor_set_torque(car.legL.behind, car.legL.TBset);
	wb_motor_set_torque(car.legL.wheel, car.legL.TWheelset);

	wb_motor_set_torque(car.legR.front, car.legR.TFset);
	wb_motor_set_torque(car.legR.behind, car.legR.TBset);
	wb_motor_set_torque(car.legR.wheel, car.legR.TWheelset);
}

void WBCbalanceMode() {
	float L_l = car.legL.L0.now;
	float L_r = car.legR.L0.now;
	float L_l2 = L_l * L_l;
	float L_r2 = L_r * L_r;
	float L_lL_r = L_l * L_r;

	float s						= 0;
	float s_dot				= car.legVir.dis.dot;
	float psi					= car.yesense.yaw.now;
	float psi_dot			= car.yesense.yaw.dot;
	float theta_l			= car.legL.angle0.now;
	float theta_l_dot = car.legL.angle0.dot;
	float theta_r			= car.legR.angle0.now;
	float theta_r_dot = car.legR.angle0.dot;
	float phi					= -car.yesense.pitch.now;
	float phi_dot			= -car.yesense.pitch.dot;

	float K[4][10];

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 10; col++) {
			int num								 = (row * 10) + col;
			K[row][col] = Kcoeff_wbc[num][0] + Kcoeff_wbc[num][1] * L_l + Kcoeff_wbc[num][2] * L_r + Kcoeff_wbc[num][3] * L_l2 +
										Kcoeff_wbc[num][4] * L_r2 + Kcoeff_wbc[num][5] * L_lL_r;
		}
	}
	float Twl = K[0][0] * (0 - s) +
							K[0][1] * (vd - s_dot) +
							K[0][2] * (psi - psi) +
							K[0][3] * (psid - psi_dot) +
							K[0][4] * (0 - theta_l) +
							K[0][5] * (0 - theta_l_dot) +
							K[0][6] * (0 - theta_r) +
							K[0][7] * (0 - theta_r_dot) +
							K[0][8] * (0 - phi) +
							K[0][9] * (0 - phi_dot);

	float Twr = K[1][0] * (0 - s) +
							K[1][1] * (vd - s_dot) +
							K[1][2] * (psi - psi) +
							K[1][3] * (psid - psi_dot) +
							K[1][4] * (0 - theta_l) +
							K[1][5] * (0 - theta_l_dot) +
							K[1][6] * (0 - theta_r) +
							K[1][7] * (0 - theta_r_dot) +
							K[1][8] * (0 - phi) +
							K[1][9] * (0 - phi_dot);

	float Tbl = K[2][0] * (0 - s) +
							K[2][1] * (vd - s_dot) +
							K[2][2] * (psi - psi) +
							K[2][3] * (psid - psi_dot) +
							K[2][4] * (0 - theta_l) +
							K[2][5] * (0 - theta_l_dot) +
							K[2][6] * (0 - theta_r) +
							K[2][7] * (0 - theta_r_dot) +
							K[2][8] * (0 - phi) +
							K[2][9] * (0 - phi_dot);

	float Tbr = K[3][0] * (0 - s) +
							K[3][1] * (vd - s_dot) +
							K[3][2] * (psi - psi) +
							K[3][3] * (psid - psi_dot) +
							K[3][4] * (0 - theta_l) +
							K[3][5] * (0 - theta_l_dot) +
							K[3][6] * (0 - theta_r) +
							K[3][7] * (0 - theta_r_dot) +
							K[3][8] * (0 - phi) +
							K[3][9] * (0 - phi_dot);

	vd												= 0;
	// 前馈力
	car.legL.Fset					 = -61.90455385f;
	car.legR.Fset					 = -61.90455385f;
	// 补偿虚拟力
	float lfCompensate		 = car.legL.L0pid.compute(&car.legL.L0pid, car.legL.L0.now);
	float rfCompensate		 = car.legR.L0pid.compute(&car.legR.L0pid, car.legR.L0.now);
	car.legL.Fset					-= lfCompensate;
	car.legR.Fset					-= rfCompensate;
	// 翻滚角补偿
	float rollCompensate	 = car.rollpid.compute(&car.rollpid, car.yesense.roll.now);
	car.legL.Fset					-= rollCompensate;
	car.legR.Fset					+= rollCompensate;

	car.legL.TWheelset			= Twl;
	car.legR.TWheelset			= Twr;

	car.legL.Tpset					 = Tbl;
	car.legR.Tpset					 = Tbr;

	VMC(&car.legL);
	VMC(&car.legR);

	car.legL.TBset *= -1;
	car.legR.TBset *= -1;

	limitInRange(float)(&car.legL.TFset, 22);
	limitInRange(float)(&car.legL.TBset, 22);
	limitInRange(float)(&car.legL.TWheelset, 10);

	limitInRange(float)(&car.legR.TFset, 22);
	limitInRange(float)(&car.legR.TBset, 22);
	limitInRange(float)(&car.legR.TWheelset, 10);

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
static float time = 0;
void				 jumpMode() {
	car.legL.L0pid.target = 0.37;
	car.legR.L0pid.target = 0.37;
	if (time < kickTime) {
		float k					= time / kickTime;
		float setpointx = jumpPoint[0][0] * (1 - k) + jumpPoint[1][0] * k;
		float setpointy = jumpPoint[0][1] * (1 - k) + jumpPoint[1][1] * k;
		Njie(&car.legL, setpointx, setpointy);
		Njie(&car.legR, setpointx, setpointy);
	} else if (time < kickTime + shrinkTime) {
		float k					= (time - kickTime) / shrinkTime;
		float setpointx = jumpPoint[1][0] * (1 - k) + jumpPoint[0][0] * k;
		float setpointy = jumpPoint[1][1] * (1 - k) + jumpPoint[0][1] * k;
		Njie(&car.legL, setpointx, setpointy);
		Njie(&car.legR, setpointx, setpointy);
	} else {
		time		 = 0;
		car.mode = ROBOTNORMAL;
		return;
	}
	time += dt;
	wb_motor_set_position(car.legL.front, car.legL.angle1set);
	wb_motor_set_position(car.legL.behind, car.legL.angle4set);
	wb_motor_set_position(car.legR.front, car.legL.angle1set);
	wb_motor_set_position(car.legR.behind, car.legL.angle4set);
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
	float lp						 = car.legL.Fnow * cos(car.legL.angle0.now) + car.legL.Tpnow * sin(car.legL.angle0.now) / car.legL.L0.now;
	float rp						 = car.legR.Fnow * cos(car.legR.angle0.now) + car.legR.Tpnow * sin(car.legR.angle0.now) / car.legR.L0.now;

	float zmdd					 = car.yesense.accelz * cos(car.yesense.pitch.now) - car.yesense.accelx * sin(car.yesense.pitch.now);

	float lzwdd					 = zmdd - car.legL.L0.ddot * cos(car.legL.angle0.now) + 2 * car.legL.L0.dot + car.legL.angle0.dot * sin(car.legL.angle0.now) + car.legL.L0.now * car.legL.angle0.ddot * sin(car.legL.angle0.now) + car.legL.L0.now * car.legL.angle0.dot * car.legL.angle0.dot * cos(car.legL.angle0.now);
	float rzwdd					 = zmdd - car.legR.L0.ddot * cos(car.legR.angle0.now) + 2 * car.legR.L0.dot + car.legR.angle0.dot * sin(car.legR.angle0.now) + car.legR.L0.now * car.legR.angle0.ddot * sin(car.legR.angle0.now) + car.legR.L0.now * car.legR.angle0.dot * car.legR.angle0.dot * cos(car.legR.angle0.now);

	car.legR.normalforce = -rp + MASSWHEEL * (GRAVITY + rzwdd);
	car.legL.normalforce = -lp + MASSWHEEL * (GRAVITY + lzwdd);

	car.force						 = (car.legL.normalforce + car.legR.normalforce) / 2;

	if (car.force < 20)
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
		case ROBOTWBC:
			WBCbalanceMode();
			break;
		default:
			break;
	}
}

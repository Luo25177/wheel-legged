#include "ADRC2.h"

#define TimeStep 10

float r_x = 1;
float r_theta = 1;
float r_phi = 1;

float b_11 = 1;
float b_12 = 1;
float b_13 = 1;
float b_21 = 1;
float b_22 = 1;
float b_23 = 1;

float w_o_1 = 9;
float w_o_2 = 8;
float w_o_3 = 12;

float w_c_11 = 3;
float w_c_12 = 3;
float w_c_13 = 5;
float w_c_21 = 3;
float w_c_22 = 3;
float w_c_23 = 5;

LTDOutput ltd = { 0 };
float dt = 0.01;
LTDOutput LTD(float x_r, float theta_r, float phi_r) {
	float x_now = dt * ltd.x_dot;
	ltd.x_dot = dt * (-r_x * r_x * ltd.x - 2 * r_x * ltd.x_dot + r_x * r_x * x_r) + ltd.x_dot;
	ltd.x = x_now;

	float theta_now = dt * ltd.theta_dot;
	ltd.theta_dot = dt * (-r_theta * r_theta * ltd.theta - 2 * r_theta * ltd.theta_dot + r_theta * r_theta * theta_r) + ltd.theta_dot;
	ltd.theta = theta_now;

	float phi_now = dt * ltd.phi_dot;
	ltd.phi_dot = dt * (-r_phi * r_phi * ltd.phi - 2 * r_phi * ltd.phi_dot + r_phi * r_phi * phi_r) + ltd.phi_dot;
	ltd.phi = phi_now;

	return ltd;
}

LESOOutput leso = { 0 };
LESOOutput LESO(float x, float theta, float phi, float Tw, float Tb) {
	float beta_11 = 3 * w_o_1;
	float beta_12 = 3 * w_o_1 * w_o_1;
	float beta_13 = w_o_1 * w_o_1 * w_o_1;
	
	float x_now = leso.x + dt * (-beta_11 * leso.x + leso.x_dot + beta_11 * x);
	float x_dot_now = leso.x_dot + dt * (-beta_12 * leso.x + leso.x_ddot + beta_12 * x + b_11 * Tw + b_21 * Tb);
	leso.x_ddot = leso.x_ddot + dt * (-beta_13 * leso.x + beta_13 * x);
	leso.x = x_now;
	leso.x_dot = x_dot_now;

	float beta_21 = 3 * w_o_2;
	float beta_22 = 3 * w_o_2 * w_o_2;
	float beta_23 = w_o_2 * w_o_2 * w_o_2;

	float theta_now = leso.theta + dt * (-beta_21 * leso.theta + leso.theta_dot + beta_21 * theta);
	float theta_dot_now = leso.theta_dot + dt * (-beta_22 * leso.theta + leso.theta_ddot + beta_22 * theta + b_12 * Tw + b_22 * Tb);
	leso.theta_ddot = leso.theta_ddot + dt * (-beta_23 * leso.theta + beta_23 * theta);
	leso.theta = theta_now;
	leso.theta_dot = theta_dot_now;

	float beta_31 = 3 * w_o_3;
	float beta_32 = 3 * w_o_3 * w_o_3;
	float beta_33 = w_o_3 * w_o_3 * w_o_3;

	float phi_now = leso.phi + dt * (-beta_31 * leso.phi + leso.phi_dot + beta_31 * phi);
	float phi_dot_now = leso.phi_dot + dt * (-beta_32 * leso.phi + leso.phi_ddot + beta_32 * phi + b_13 * Tw + b_23 * Tb);
	leso.phi_ddot = leso.phi_ddot + dt * (-beta_33 * leso.phi + beta_33 * phi);
	leso.phi = phi_now;
	leso.phi_dot = phi_dot_now;

	return leso;
}

LSEFOutput lsef = { 0 };
LSEFOutput LSEF() {
	float a_111 = w_c_11 * 2;
	float a_112 = w_c_11 * w_c_11;
	float a_121 = w_c_12 * 2;
	float a_122 = w_c_12 * w_c_12;
	float a_131 = w_c_13 * 2;
	float a_132 = w_c_13 * w_c_13;
	
	float a_211 = w_c_21 * 2;
	float a_212 = w_c_21 * w_c_21;
	float a_221 = w_c_22 * 2;
	float a_222 = w_c_22 * w_c_22;
	float a_231 = w_c_23 * 2;
	float a_232 = w_c_23 * w_c_23;

	lsef.Tw = (a_111 * (ltd.x - leso.x) + a_112 * (ltd.x_dot - leso.x_dot) - leso.x_ddot) / b_11 +
		(a_121 * (ltd.theta - leso.theta) + a_122 * (ltd.theta_dot - leso.theta_dot) - leso.theta_ddot) / b_12 +
		(a_131 * (ltd.phi - leso.phi) + a_132 * (ltd.phi_dot - leso.phi_dot) - leso.phi_ddot) / b_13;

	lsef.Tb = (a_211 * (ltd.x - leso.x) + a_212 * (ltd.x_dot - leso.x_dot) - leso.x_ddot) / b_21 +
		(a_221 * (ltd.theta - leso.theta) + a_222 * (ltd.theta_dot - leso.theta_dot) - leso.theta_ddot) / b_22 +
		(a_231 * (ltd.phi - leso.phi) + a_232 * (ltd.phi_dot - leso.phi_dot) - leso.phi_ddot) / b_23;

	return lsef;
}




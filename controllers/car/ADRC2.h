#pragma once

typedef struct {
	float x;
	float x_dot;
	float theta;
	float theta_dot;
	float phi;
	float phi_dot;
} LTDOutput;

typedef struct {
	float x;
	float x_dot;
	float x_ddot;
	float theta;
	float theta_dot;
	float theta_ddot;
	float phi;
	float phi_dot;
	float phi_ddot;
} LESOOutput;

typedef struct {
	float Tw;
	float Tb;
} LSEFOutput;

LTDOutput LTD(float x_r, float theta_r, float phi_r);
LESOOutput LESO(float x, float theta, float phi, float Tw, float Tb);
LSEFOutput LSEF();



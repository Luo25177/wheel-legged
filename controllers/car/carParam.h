#pragma once

typedef struct {
	float x;
	float x_dot;
	float theta;
	float theta_dot;
	float phi;
	float phi_dot;
} SysState;

typedef struct {
	float Tw;
	float Tp;
} SysInput;


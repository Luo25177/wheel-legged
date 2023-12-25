#pragma once
//----
// @file datastruct.h
// @author mask (beloved25177@126.com)
// @brief
// @version 1.0
// @date 2023-11-12
//
// @copyright Copyright (c) 2023
// @details this is a datastruct to storage some wheelfoot data
//----

typedef struct {
	float now;
	float dot;
	float ddot;
	float last;
	float lastdot;
	float set;
	float setdot;
} datastruct;

void datastructInit(datastruct* data, const float _now, const float _last, const float _dot, const float _ddot);

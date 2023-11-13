//----
// @file leg.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-12
// 
// @copyright Copyright (c) 2023
// 
//----

#pragma once

#include "djmotor.h"
#include "tmotor.h"



typedef struct {
  DJmotor* wheel;
  Tmotor* front, behind;
}Leg;

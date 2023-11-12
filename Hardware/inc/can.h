//----
// @file can.h
// @author mask (beloved25177@126.com)
// @brief 
// @version 1.0
// @date 2023-11-12
// 
// @copyright Copyright (c) 2023
// 
//----

//! RTR一定得设置，不然会发生id错误的情况，并且数据不对
#pragma once

#include "misc.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void can1Init();
void can2Init();

void can1Check();
void can2Check();

void canSend(u8 ctrlWord);
void sendzero();
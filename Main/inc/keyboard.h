//----
// @file keyboard.h
// @author mask <beloved25177@126.com>
// @brief
// @version 1.0
// @date 2023-11-24
//
// @copyright Copyright (c) 2023
//
//----
#pragma once

#include "beep.h"
#include "bluetooth.h"
#include "serialscreen.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void keyboardInit();

u8	 leftboardXscan(void);
u8	 leftboardYscan(void);
u8	 rightboardXscan(void);
u8	 rightboardYscan(void);
void readLeftBoard();
void readRightBoard();

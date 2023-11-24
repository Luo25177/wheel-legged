//----
// @file heandle.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-24
// 
// @copyright Copyright (c) 2023
// 
//----
#pragma once

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "includes.h"
#include "masterparam.h"

void handleInit(void);
void getHandleADC(void);


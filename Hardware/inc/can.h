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

void Can1Init();
void Can2Init();

void Can1Check();
void Can2Check();

void CanSend(u8 ctrlWord);
void CanCheck();

extern queue(CanTxMsg) * can1Txmsg;
extern queue(CanRxMsg) * can1Rxmsg;
extern queue(CanTxMsg) * can2Txmsg;
extern queue(CanRxMsg) * can2Rxmsg;

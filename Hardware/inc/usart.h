/**
 * @file usart.h
 * @author
 * @brief 蓝牙通信 u1控制器 u2yesense
 * @version 0.1
 * @date 2023-08-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "misc.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

typedef struct Usart {
	u8 rxBuff[128];
	u8 txBuff[128];
	u8 dmaAble;
	void (*send)(u8* data, u8 cnt);
} Usart;

void usart1Init();
void usart2Init();

void usart1Send(u8* data, u8 cnt);
void usart2Send(u8* data, u8 cnt);

extern Usart* usart1;
extern Usart* usart2;

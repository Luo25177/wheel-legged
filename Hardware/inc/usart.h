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
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "string.h"

#define MAXUSARTDATASIZE 70

typedef struct Usart {
  u8 rxBuff[MAXUSARTDATASIZE];
  u8 txBuff[MAXUSARTDATASIZE];
  void (*send)(u8* data, u8 cnt);
} Usart;

void Usart1Init();
void Usart2Init();
void Usart3Init();
void Uart5Init();

void Usart1Send(u8* data, u8 cnt);
void Usart2Send(u8* data, u8 cnt);

extern Usart* usart1;
extern Usart* usart2;

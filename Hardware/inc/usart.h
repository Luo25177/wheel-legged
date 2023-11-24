/**
 * @file usart.h
 * @author your name (you@domain.com)
 * @brief u1 串口屏 u2 蓝牙主控
 * @version 0.1
 * @date 2023-08-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef struct Usart
{
    u8 rxBuff[128];
    u8 txBuff[128];
    u8 dmaAble;
    void (*send)(u8 *data, u8 cnt);

} Usart;

void usart1Init();

void usart2Init();

void usart1Send(u8 *data, u8 cnt);
void usart1SendSerial(u8 cnt);
void usart2Send(u8 *data, u8 cnt);

extern Usart *usart1;
extern Usart *usart2;

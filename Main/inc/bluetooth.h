//----
// @file bluetooth.h
// @author mask <beloved25177@126.com>
// @brief 蓝牙通讯，消息处理
// @version 1.0
// @date 2023-11-17
//
// @copyright Copyright (c) 2023
//
//----
#pragma once

#include "masterparam.h"
#include "stm32f4xx.h"
#include "usart.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define HEADCHAR1 0xff
#define HEADCHAR2 0xfe
#define TAILCHAR1 0x0a
#define TAILCHAR2 0x0d

#define BLUETOOTHDATALEN 50

typedef struct {
  u8 rxData[BLUETOOTHDATALEN];
  u8 txData[BLUETOOTHDATALEN];
  u8 head[2];
  u8 tail[2];
  u8 rxDataSize;
  u8 txDataSize;

  bool gethead;
} BlueToothMsg;

extern BlueToothMsg* bluetoothmsg;

void blueToothInit();
void blueToothReceive(u8 data);
void blueToothDeal();
void blueToothClear();
void blueToothSend(u8 id, void* data, u8 size);

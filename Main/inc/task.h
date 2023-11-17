//----
// @file task.h
// @author mask (beloved25177@126.com)
// @brief 
// @version 1.0
// @date 2023-11-12
// 
// @copyright Copyright (c) 2023
// 
//----

#pragma once

#include "led.h"
#include "beep.h"
#include "usart.h"
#include "os_cfg.h"
#include "os_cpu.h"
#include "ucos_ii.h"

extern OS_EVENT* beepShowSem;
extern u8 beepShowErr;

#define TASK_STK_SIZE 256

#define START_TASK_PRIO 5	   //
static void taskStart(void* pdata);

#define LED_TASK_PRIO	12
static void taskLed(void* pdata);

#define BEEP_TASK_PRIO 11
static void taskBeep(void* pdata);


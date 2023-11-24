//----
// @file main.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-24
// 
// @copyright Copyright (c) 2023
// 
//----
#pragma once

#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "misc.h"
#include "led.h"
#include "tim.h"
#include "usart.h"
#include "serialscreen.h"
#include "keyboard.h"
#include "delay.h"
#include "handle.h"
#include "bluetooth.h"

#define TASK_STK_SIZE 256

#define START_TASK_PRIO 5
OS_STK taskStartStk[TASK_STK_SIZE];
static void taskStart(void *pdata);

#define BEEP_TASK_PRIO 11
OS_STK taskBeepStk[TASK_STK_SIZE];
static void taskBeep(void *pdata);

#define LED_TASK_PRIO 10
OS_STK taskLedStk[TASK_STK_SIZE];
static void taskLed(void *pdata);

#define MSG_TASK_PRIO 6
OS_STK taskMsgStk[TASK_STK_SIZE];
static void taskMsg(void *pdata);


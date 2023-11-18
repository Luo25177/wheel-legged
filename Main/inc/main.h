//----
// @file main.h
// @author mask <beloved25177@126.com>
// @brief main函数，主程序
// @version 1.0
// @date 2023-11-17
//
// @copyright Copyright (c) 2023
//
//----
#pragma once

#include "can.h"
#include "tim.h"
#include "led.h"
#include "beep.h"
#include "robot.h"
#include "usart.h"
#include "os_cfg.h"
#include "os_cpu.h"
#include "ucos_ii.h"
#include "masterparam.h"

#define TASK_STK_SIZE 256

OS_CPU_SR cpu_sr = 0;
OS_STK taskStartStk[TASK_STK_SIZE];
OS_STK taskLedStk[TASK_STK_SIZE];
OS_STK taskBeepStk[TASK_STK_SIZE];

OS_EVENT *beepShowSem;
u8 beepShowErr;

extern u8 beepShowErr;
extern OS_EVENT *beepShowSem;
extern OS_STK taskStartStk[TASK_STK_SIZE];

#define START_TASK_PRIO 5
static void taskStart(void *pdata);

#define LED_TASK_PRIO 12
static void taskLed(void *pdata);

#define BEEP_TASK_PRIO 11
static void taskBeep(void *pdata);

#define RUN_TASK_PRIO 6
static void taskRun(void *pdata);
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

#include "beep.h"
#include "bluetooth.h"
#include "can.h"
#include "led.h"
#include "masterparam.h"
#include "os_cfg.h"
#include "os_cpu.h"
#include "robot.h"
#include "robotmonitor.h"
#include "tim.h"
#include "ucos_ii.h"
#include "usart.h"

#define TASK_STK_SIZE 256

OS_CPU_SR cpu_sr = 0;

u8 beepShowErr;
OS_EVENT *beepShowSem;

u8 seekZeroErr;
OS_EVENT *seekZeroSem;

OS_STK taskRunStk[TASK_STK_SIZE];
OS_STK taskLedStk[TASK_STK_SIZE];
OS_STK taskBeepStk[TASK_STK_SIZE];
OS_STK taskStartStk[TASK_STK_SIZE];
OS_STK taskTestStk[TASK_STK_SIZE];
OS_STK taskInitStk[TASK_STK_SIZE];

#define START_TASK_PRIO 5
static void TaskStart(void *pdata);

#define LED_TASK_PRIO 12
static void TaskLed(void *pdata);

#define BEEP_TASK_PRIO 11
static void TaskBeep(void *pdata);

#define RUN_TASK_PRIO 6
static void TaskRun(void *pdata);

#define INIT_TASK_PRIO 8
static void TaskInit(void *pdata);

#define TEST_TASK_PRIO 7
static void TaskTest(void *pdata);

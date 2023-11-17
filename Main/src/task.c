#include "task.h"

OS_CPU_SR cpu_sr = 0;
OS_STK taskStartStk[TASK_STK_SIZE];
OS_STK taskLedStk[TASK_STK_SIZE];
OS_STK	taskBeepStk[TASK_STK_SIZE];

OS_EVENT* beepShowSem;
u8 beepShowErr;

static void taskStart(void* pdata) {
	pdata = pdata;

	OS_CPU_SysTickInit();	 // 重要！！！不开启无法进行任务调度 启动ucosii的时钟
	OS_ENTER_CRITICAL();	 // 程序进入临界段，无法被中断打断

  beepShowSem = OSSemCreate(0);
  // runSem		= OSSemCreate(0);
  // manualSem	= OSSemCreate(0);
  // initSem		= OSSemCreate(0);
  // disAbleSem	= OSSemCreate(0);

	OSTaskCreate(taskLed, (void*) 0, &taskLedStk[TASK_STK_SIZE - 1], LED_TASK_PRIO);
	OSTaskCreate(taskBeep, (void*) 0, &taskBeepStk[TASK_STK_SIZE - 1], BEEP_TASK_PRIO);

	OS_EXIT_CRITICAL();				   // 程序退出临界段，可以被中断打断，在临界段中不要加延时，会死机
	OSTaskSuspend(START_TASK_PRIO);	   // 根据程序优先级挂起起始任务 每个任务单独一个优先级
}

static void taskLed(void* pdata) {
	pdata = pdata;
	while (1) {
		ledShow();
		OSTimeDly(100);
	}
}

static void taskBeep(void* pdata) {
	pdata = pdata;
	beepShow(3);
	while (1) {
		beepShowSem->OSEventCnt = 0;
		OSSemPend(beepShowSem, 0, &beepShowErr);
		beepShow(1);
		OSTimeDly(100);
	}
}
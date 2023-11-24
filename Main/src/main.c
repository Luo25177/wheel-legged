#include "main.h" 

OS_CPU_SR cpu_sr = 0;

int main() {
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	ledInit();
	beepInit();
	usart1Init();
	usart2Init();
	tim2Init();
	tim3Init();
	keyboardInit();
	serialInit(&serial);

	beepShow(2);

	OSInit();																						// 初始化UCOSII
	OSTaskCreate(taskStart, (void*) 0, &taskStartStk[TASK_STK_SIZE - 1], START_TASK_PRIO);	// 创建初始任务
	OSStart();
}

static void taskStart(void* pdata)
{
	pdata = pdata;

	OS_CPU_SysTickInit();	 // 重要！！！不开启无法进行任务调度 启动ucosii的时钟

	beepShow(3);
	OS_ENTER_CRITICAL();	// 程序进入临界段，无法被中断打断

	beepShowSem = OSSemCreate(0);
	// dealSerialMsgSem = OSSemCreate(0);

	OSTaskCreate(taskLed, (void*) 0, &taskLedStk[TASK_STK_SIZE - 1], LED_TASK_PRIO);
	OSTaskCreate(taskBeep, (void*) 0, &taskBeepStk[TASK_STK_SIZE - 1], BEEP_TASK_PRIO);
	OSTaskCreate(taskMsg, (void *)0, &taskMsgStk[TASK_STK_SIZE - 1], MSG_TASK_PRIO);

	OS_EXIT_CRITICAL();				   // 程序退出临界段，可以被中断打断，在临界段中不要加延时，会死机
	OSTaskSuspend(START_TASK_PRIO);	   // 挂起起始任务}
}

static void taskLed(void* pdata)
{
	pdata = pdata;
	while (1) {
		ledShow();
		OSTimeDly(1000);
	}
}

static void taskBeep(void* pdata)
{
	pdata = pdata;
	while (1) {
		beepShowSem->OSEventCnt = 0;
		OSSemPend(beepShowSem, 0, &beepShowErr);
		beepShow(1);
		OSTimeDly(1000);
	}
}

static void taskMsg(void *pdata) {
  pdata = pdata;
  while(1) {
    getHandleADC();
    readLeftBoard();
    readRightBoard();
		serialUpdate(&serial, usart1->txBuff);
    OSTimeDly(100);
  }
}

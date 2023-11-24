#include "main.h"

int main(){
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	ledInit();
	beepInit();
	usart1Init();
	usart2Init();
	can1Init();
	can2Init();
	tim2Init();
	tim3Init();

  robotInit();
	MasterInit(&master);
	RobotStateInit(&robotstate);

	OSInit();
	OSTaskCreate(taskStart, (void*) 0, &taskStartStk[TASK_STK_SIZE - 1], START_TASK_PRIO);	// 创建初始任务
	OSStart(); 
}

static void taskStart(void* pdata) {
	pdata = pdata;

	OS_CPU_SysTickInit();	 // 重要！！！不开启无法进行任务调度 启动ucosii的时钟
	OS_ENTER_CRITICAL();	 // 程序进入临界段，无法被中断打断

  beepShowSem = OSSemCreate(0);

	OSTaskCreate(taskLed, (void*) 0, &taskLedStk[TASK_STK_SIZE - 1], LED_TASK_PRIO);
	OSTaskCreate(taskBeep, (void*) 0, &taskBeepStk[TASK_STK_SIZE - 1], BEEP_TASK_PRIO);
	OSTaskCreate(taskRun, (void*) 0, &taskRunStk[TASK_STK_SIZE - 1], BEEP_TASK_PRIO);

	OS_EXIT_CRITICAL();				   // 程序退出临界段，可以被中断打断，在临界段中不要加延时，会死机
	OSTaskSuspend(START_TASK_PRIO);	   // 根据程序优先级挂起起始任务 每个任务单独一个优先级
}

static void taskLed(void* pdata) {
	pdata = pdata;
	while (1) {
		ledShow();
		OSTimeDly(5000);
	}
}

static void taskBeep(void* pdata) {
	pdata = pdata;
	beepShow(3);
	while (1) {
		beepShowSem->OSEventCnt = 0;
		OSSemPend(beepShowSem, 0, &beepShowErr);
		beepShow(1);
		OSTimeDly(1000);
	}
}

static void taskRun(void* pdata) {
	pdata = pdata;
	while (1) {
		updateState();
		if(master.control.begin){
			robotRun();
		}
		OSTimeDly(100);
	}
}

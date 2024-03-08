#include "main.h"

int main() {
  SystemInit();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

  LedInit();
  BeepInit();
  Usart1Init();
  Usart2Init();
  Can1Init();
  Can2Init();
  Tim2Init();
  Tim3Init();

  RobotInit();
  BlueToothInit();

  OSInit();
  OSTaskCreate(TaskStart, (void*) 0, &taskStartStk[TASK_STK_SIZE - 1], START_TASK_PRIO);  // 创建初始任务
  OSStart();
}

static void TaskStart(void* pdata) {
  pdata = pdata;

  OS_CPU_SysTickInit();  //! 重要！！！不开启无法进行任务调度 启动ucosii的时钟
  OS_ENTER_CRITICAL();   // 程序进入临界段，无法被中断打断

  beepShowSem = OSSemCreate(0);

  OSTaskCreate(TaskLed, (void*) 0, &taskLedStk[TASK_STK_SIZE - 1], LED_TASK_PRIO);
  //  OSTaskCreate(TaskBeep, (void*) 0, &taskBeepStk[TASK_STK_SIZE - 1], BEEP_TASK_PRIO);
  OSTaskCreate(TaskRun, (void*) 0, &taskRunStk[TASK_STK_SIZE - 1], RUN_TASK_PRIO);
  //  OSTaskCreate(TaskInit, (void*) 0, &taskInitStk[TASK_STK_SIZE - 1], INIT_TASK_PRIO);
  OSTaskCreate(TaskTest, (void*) 0, &taskTestStk[TASK_STK_SIZE - 1], TEST_TASK_PRIO);

  OS_EXIT_CRITICAL();              //! 程序退出临界段，可以被中断打断，在临界段中不要加延时，会死机
  OSTaskSuspend(START_TASK_PRIO);  // 根据程序优先级挂起起始任务 每个任务单独一个优先级
}

//----
// @brief 流水灯
//
// @param pdata
//----
static void TaskLed(void* pdata) {
  pdata = pdata;
  while (1) {
    LedShow();
    OSTimeDly(3000);
  }
}

//----
// @brief 蜂鸣器
//
// @param pdata
//----
static void TaskBeep(void* pdata) {
  pdata = pdata;
  BeepShow(3);
  while (1) {
    beepShowSem->OSEventCnt = 0;
    OSSemPend(beepShowSem, 0, &beepShowErr);
    BeepShow(1);
    OSTimeDly(1000);
  }
}

//----
// @brief 用于关节电机的初始化和寻零点
//
// @param pdata
//----
static void TaskInit(void* pdata) {
  pdata = pdata;
  while (1) {
    seekZeroSem->OSEventCnt = 0;
    OSSemPend(seekZeroSem, 0, &seekZeroErr);
    while (!(TmotorSeekZero(robot.legL.front, TSEEKZEROSPEED) &&
             TmotorSeekZero(robot.legL.behind, TSEEKZEROSPEED) &&
             TmotorSeekZero(robot.legR.front, TSEEKZEROSPEED) &&
             TmotorSeekZero(robot.legR.behind, TSEEKZEROSPEED)))
      OSTimeDly(100);
    BeepShow(2);
    OSTimeDly(10000);
  }
}

//----
// @brief 开始运行 	0.67215ms
//
// @param pdata
//----
static void TaskRun(void* pdata) {
  pdata = pdata;
  while (1) {
    UpdateState();
    if (master.control.begin) {
      tmotor[0].monitor.enable  = true;
      tmotor[1].monitor.enable  = true;
      djmotor[0].monitor.enable = true;
      tmotor[2].monitor.enable  = true;
      tmotor[3].monitor.enable  = true;
      djmotor[1].monitor.enable = true;
      RobotRun();
    }
    OSTimeDly(20);
  }
}

//----
// @brief 测试任务
//
// @param pdata
//----
static void TaskTest(void* pdata) {
  pdata = pdata;
  while (1) {
    TmotorRun(tmotor);
    DJmotorRun(djmotor);
    OSTimeDly(10);
  }
}

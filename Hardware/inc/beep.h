#pragma once

#include "os_cfg.h"
#include "os_cpu.h"
#include "ucos_ii.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

extern OS_EVENT* beepShowSem;
extern u8 beepShowErr;

#define BEEP_OFF GPIOA->BSRRH = GPIO_Pin_8
#define BEEP_ON	 GPIOA->BSRRL = GPIO_Pin_8

void beepInit();
void beepShow(int num);
void beepWarning();

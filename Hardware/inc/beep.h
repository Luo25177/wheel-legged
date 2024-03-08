#pragma once

#include "os_cfg.h"
#include "os_cpu.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "ucos_ii.h"

extern OS_EVENT* beepShowSem;
extern u8        beepShowErr;

#define BEEP_OFF GPIOA->BSRRH = GPIO_Pin_8
#define BEEP_ON  GPIOA->BSRRL = GPIO_Pin_8

void BeepInit();
void BeepShow(int num);
void BeepWarning();

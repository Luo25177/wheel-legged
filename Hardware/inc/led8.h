#pragma once

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define Set_LED8CLK    GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define Reset_LED8CLK  GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define Set_LED8DATA   GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define Reset_LED8DATA GPIO_ResetBits(GPIOB, GPIO_Pin_9)

void Led8Init(void);
void Led8Show(unsigned char Data);

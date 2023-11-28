#pragma once

#include "bluetooth.h"
#include "keyboard.h"
#include "serialScreen.h"
#include "usart.h"

void DMA2_Stream7_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

//----
// @file irqhandler.h
// @author mask <beloved25177@126.com>
// @brief 
// @version 1.0
// @date 2023-11-13
// 
// @copyright Copyright (c) 2023
// @details 对于一些消息接收中断的处理
//----
#pragma once

#include "can.h"
#include "tim.h"
#include "usart.h"
#include "robot.h"

void DMA2_Stream7_IRQHandler();
void DMA1_Stream6_IRQHandler();

void USART1_IRQHandler();
void USART2_IRQHandler();

void TIM2_IRQHandler();
void TIM3_IRQHandler();

void CAN1_RX0_IRQHandler();
void CAN2_RX0_IRQHandler();

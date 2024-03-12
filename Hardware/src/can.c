#include "can.h"

queue(CanTxMsg) * can1Txmsg;
queue(CanRxMsg) * can1Rxmsg;
queue(CanTxMsg) * can2Txmsg;
queue(CanRxMsg) * can2Rxmsg;

void Can1Init() {
  NVIC_InitTypeDef      NVIC_InitStructure;
  CAN_InitTypeDef       CAN_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  CAN_InitStructure.CAN_TTCM      = DISABLE;          // 非时间触发通道模式
  CAN_InitStructure.CAN_ABOM      = DISABLE;          // 软件对CAN_MCR寄存器的INRQ位置1，随后清0，一旦监测到128次连续11位的隐性位，就退出离线状态
  CAN_InitStructure.CAN_AWUM      = DISABLE;          // 睡眠模式由软件唤醒
  CAN_InitStructure.CAN_NART      = DISABLE;          // 禁止报文自动发送，即只发送一次，无论结果如何
  CAN_InitStructure.CAN_RFLM      = DISABLE;          // 报文不锁定，新的覆盖旧的
  CAN_InitStructure.CAN_TXFP      = DISABLE;          // 发送FIFO的优先级由标识符决定
  CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal;  // CAN硬件工作在正常模式

  CAN_InitStructure.CAN_SJW       = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1       = CAN_BS1_9tq;
  CAN_InitStructure.CAN_BS2       = CAN_BS2_4tq;
  CAN_InitStructure.CAN_Prescaler = 3;
  CAN_Init(CAN1, &CAN_InitStructure);

  CAN_FilterInitStructure.CAN_FilterNumber         = 1;
  CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x00 << 5;
  CAN_FilterInitStructure.CAN_FilterIdLow          = 0x02 << 5;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0x03 << 5;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x04 << 5;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);  // FIFO 0 message pending Interrupt
  CAN_ClearFlag(CAN1, CAN_IT_FMP0);
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

  CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);  // Error:Bus-off Interrupt
  CAN_ClearFlag(CAN1, CAN_IT_BOF);
  CAN_ITConfig(CAN1, CAN_IT_BOF, ENABLE);

  can1Txmsg = newqueue(CanTxMsg)(QUEUEMAXSIZE);
  can1Rxmsg = newqueue(CanRxMsg)(QUEUEMAXSIZE);
}

void Can2Init() {
  NVIC_InitTypeDef      NVIC_InitStructure;
  CAN_InitTypeDef       CAN_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM      = DISABLE;          // 非时间触发通道模式
  CAN_InitStructure.CAN_ABOM      = ENABLE;          // 软件对CAN_MCR寄存器的INRQ位置1，随后清0，一旦监测到128次连续11位的隐性位，就退出离线状态
  CAN_InitStructure.CAN_AWUM      = DISABLE;          // 睡眠模式由软件唤醒
  CAN_InitStructure.CAN_NART      = DISABLE;          // 禁止报文自动发送，即只发送一次，无论结果如何
  CAN_InitStructure.CAN_RFLM      = DISABLE;          // 报文不锁定，新的覆盖旧的
  CAN_InitStructure.CAN_TXFP      = DISABLE;          // 发送FIFO的优先级由标识符决定
  CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal;  // CAN硬件工作在正常模式

  /* Seting BaudRate */
  CAN_InitStructure.CAN_SJW       = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1       = CAN_BS1_9tq;
  CAN_InitStructure.CAN_BS2       = CAN_BS2_4tq;
  CAN_InitStructure.CAN_Prescaler = 3;
  CAN_Init(CAN2, &CAN_InitStructure);

  CAN_FilterInitStructure.CAN_FilterNumber         = 14;
  CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x001 << 5;
  CAN_FilterInitStructure.CAN_FilterIdLow          = 0x002 << 5;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0x203 << 5;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x204 << 5;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
  CAN_ClearFlag(CAN2, CAN_IT_FMP0);
  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

  CAN_ClearITPendingBit(CAN2, CAN_IT_BOF);
  CAN_ClearFlag(CAN2, CAN_IT_BOF);
  CAN_ITConfig(CAN2, CAN_IT_BOF, ENABLE);

  can2Txmsg = newqueue(CanTxMsg)(QUEUEMAXSIZE);
  can2Rxmsg = newqueue(CanRxMsg)(QUEUEMAXSIZE);
}

//----
// @brief 发送can消息，使用标志位 0000 0011 can2 can1
//
// @param ctrlWord
//----
void CanSend(u8 ctrlWord) {
  CanTxMsg msg;
  int      i = 2;  // 每次发送两条控制消息
  while (i--) {
    if ((ctrlWord & 0x01) && can1Txmsg->pop(can1Txmsg, &msg))
      CAN_Transmit(CAN1, &msg);
    if ((ctrlWord & 0x02) && can2Txmsg->pop(can2Txmsg, &msg))
      CAN_Transmit(CAN2, &msg);
  }
}

void CanCheck() { }

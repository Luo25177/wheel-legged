#include "usart.h"

Usart* usart1;
Usart* usart2;

void usart1Init() {
	usart1					= (Usart*) malloc(sizeof(Usart));
	usart1->send		= usart1Send;
	usart1->dmaAble = 1;

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	DMA_InitTypeDef		DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate						= 115200;
	USART_InitStructure.USART_WordLength					= USART_WordLength_8b;
	USART_InitStructure.USART_stopBits						= USART_stopBits_1;
	USART_InitStructure.USART_Parity							= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode								= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel									 = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority				 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd								 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(DMA2_Stream7);
	DMA_InitStructure.DMA_Channel						 = DMA_Channel_4;
	DMA_InitStructure.DMA_BufferSize				 = 100;
	DMA_InitStructure.DMA_DIR								 = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr		 = (uint32_t) (usart1->txBuff);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&USART1->DR);
	DMA_InitStructure.DMA_MemoryInc					 = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc			 = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryDataSize		 = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode							 = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority					 = DMA_Priority_VeryHigh;	 // 以下为f4特有
	DMA_InitStructure.DMA_FIFOMode					 = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold			 = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_PeripheralBurst		 = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_MemoryBurst				 = DMA_MemoryBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);	// 清除中断标志
	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	// 关闭了DMA传输完成中断，导致数据无法传输。

	NVIC_InitStructure.NVIC_IRQChannel									 = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority				 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd								 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag(USART1, USART_IT_TC);						// 清除中断标志
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	// 接受中断
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

void usart2Init() {
	usart2					= (Usart*) malloc(sizeof(Usart));
	usart2->send		= usart2Send;
	usart2->dmaAble = 1;

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	DMA_InitTypeDef		DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate						= 115200;
	USART_InitStructure.USART_WordLength					= USART_WordLength_8b;
	USART_InitStructure.USART_stopBits						= USART_stopBits_1;
	USART_InitStructure.USART_Parity							= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode								= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// 配置NVIC
	NVIC_InitStructure.NVIC_IRQChannel									 = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority				 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd								 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel						 = DMA_Channel_4;
	DMA_InitStructure.DMA_BufferSize				 = 0;
	DMA_InitStructure.DMA_DIR								 = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr		 = (uint32_t) (usart2->txBuff);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&USART2->DR);
	DMA_InitStructure.DMA_MemoryInc					 = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc			 = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryDataSize		 = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode							 = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority					 = DMA_Priority_VeryHigh;	 // 以下为f4特有
	DMA_InitStructure.DMA_FIFOMode					 = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold			 = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_PeripheralBurst		 = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_MemoryBurst				 = DMA_MemoryBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);

	DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);	// 清除中断标志
	DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);	// 关闭了DMA传输完成中断，导致数据无法传输。10/14/2018

	NVIC_InitStructure.NVIC_IRQChannel									 = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority				 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd								 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag(USART2, USART_IT_TC);						// 清除中断标志
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	// 接受中断
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	// 使能USART
	USART_Cmd(USART2, ENABLE);
}

void usart3Init() {
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate						= 115200;
	USART_InitStructure.USART_WordLength					= USART_WordLength_8b;
	USART_InitStructure.USART_stopBits						= USART_stopBits_1;
	USART_InitStructure.USART_Parity							= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode								= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel									 = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority				 = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd								 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART3, ENABLE);
}

void uart5Init() {
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate						= 115200;
	USART_InitStructure.USART_WordLength					= USART_WordLength_8b;
	USART_InitStructure.USART_stopBits						= USART_stopBits_1;
	USART_InitStructure.USART_Parity							= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode								= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	 // 接收中断

	// 配置NVIC
	NVIC_InitStructure.NVIC_IRQChannel									 = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority				 = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd								 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 使能USART
	USART_Cmd(UART5, ENABLE);
}

void usart1Send(u8* data, u8 cnt) {
	memcpy(usart1->txBuff, data, cnt);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA2_Stream7, DISABLE);	 // 关闭 DMA 传输
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
		;																					// 确保 DMA 可以被设置
	DMA_SetCurrDataCounter(DMA2_Stream7, cnt);	// 数据传输量
	DMA_Cmd(DMA2_Stream7, ENABLE);
}

void usart2Send(u8* data, u8 cnt) {
	memcpy(usart2->txBuff, data, cnt);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Stream6, DISABLE);	 // 关闭 DMA 传输
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
		;																					// 确保 DMA 可以被设置
	DMA_SetCurrDataCounter(DMA1_Stream6, cnt);	// 数据传输量
	DMA_Cmd(DMA1_Stream6, ENABLE);
}

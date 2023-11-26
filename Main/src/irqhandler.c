#include "irqhandler.h"

u8	  HEAD_FLAG		   = 0;	   // 包头标志位
u8	  RX_FLAG		   = 0;
short Data_len		   = 2;
u8	  Data_Yesense[98] = {0x59, 0x53};
int	  res			   = 0;
u8	  n;
float yaw_last = 0, yaw_now = 0;

void  G_output_infoSet(Yesense* yesense) {
	yaw_last = yaw_now;
	yaw_now	 = yesense->yaw.now;
	if (yaw_now - yaw_last < -100)	  // 发生了突变
		n += 1;
	else if (yaw_now - yaw_last > 100)
		n -= 1;
	yesense->yaw.now += n * 360;
}

// 数据传输完成，产生中断，检查是否还有没有传输的数据，继续传输
void DMA2_Stream7_IRQHandler(void) { 
	if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) == SET) {
		DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);	  // 清除中断标志
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	}
}

// 数据传输完成,产生中断,检查是否还有没有传输的数据，继续传输
void DMA1_Stream6_IRQHandler(void) {
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET) {
		DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);	  // 清除中断标志
		DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
	}
}

void USART1_IRQHandler(void) {
	u8 temp;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	  // 接收寄存器非空
	{
		USART_ClearFlag(USART1, USART_IT_RXNE);	   // USART_FLAG_RXNE        //清除中断标志
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		temp = USART_ReceiveData(USART1);
		blueToothReceive(temp);
	}
}

void USART2_IRQHandler(void) {
	u8 temp;
	if (USART_GetITStatus(USART2, USART_IT_ORE_RX) != RESET) {
		temp = USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_ORE);	// 清除溢出中断
	}
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		temp = USART_ReceiveData(USART2);
		if (RX_FLAG == 1) {
			Data_len++;
			Data_Yesense[Data_len - 1] = temp;
			if (Data_len > 97)	  // 超出范围
			{
				Data_len = 2;
				RX_FLAG	 = 0;
			}
		}
		if (HEAD_FLAG == 1) {
			if (temp == 0x53)	 // 帧头2
			{
				Data_len  -= 2;
				res	= yesenseAnalyze(&robot->yesense, Data_Yesense, Data_len);
				Data_len   = 2;
				RX_FLAG	   = 1;
				HEAD_FLAG  = 0;
				if (res == 0 || res == 1)
					G_output_infoSet(&robot->yesense);
			} else if (temp != 0x59)
				HEAD_FLAG = 0;
		}
		if (temp == 0x59)	 // 帧头1
			HEAD_FLAG = 1;
	}
}
 
// 1ms 
void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)	// 溢出中断
	{
		GolbalTimer++; // 全局计时器 代码的全局时间可都靠这个
		canSend(0x3);
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	   // 清除中断标志位
}

// 100ms
void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
		RobotStateUpdate(&robotstate);
		blueToothSend(3, (void *) &robotstate, sizeof(RobotState));
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void CAN1_RX0_IRQHandler(void) {
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_ClearFlag(CAN1, CAN_IT_FMP0);
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
		CanRxMsg rxMsg;
		CAN_Receive(CAN1, CAN_FIFO0, &rxMsg);
		TmotorreceiveHandle(tmotor, rxMsg);
	}
}

void CAN2_RX0_IRQHandler(void) {
	CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	CAN_ClearFlag(CAN2, CAN_IT_FMP0);
	if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET) {
		CanRxMsg rxMsg;
		CAN_Receive(CAN2, CAN_FIFO0, &rxMsg);
		DJmotorreceiveHandle(djmotor, rxMsg);
	}
}

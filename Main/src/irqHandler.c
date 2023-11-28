#include "irqHandler.h"

void DMA2_Stream7_IRQHandler(void)	// 数据传输完成，产生中断，检查是否还有没有传输的数据，继续传输
{
	if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) == SET) {
		DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);	// 清除中断标志
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	}
}

void DMA1_Stream6_IRQHandler(void)	// 数据传输完成,产生中断,检查是否还有没有传输的数据，继续传输
{
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET) {
		DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);	// 清除中断标志
		DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
	}
}

void USART1_IRQHandler(void) {
	u8 temp;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	// 接收寄存器非空
	{
		USART_ClearFlag(USART1, USART_IT_RXNE);	 // USART_FLAG_RXNE        //清除中断标志
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		temp = USART_ReceiveData(USART1);
		serialReceiveHandler(&serial, temp);
	}
}

void USART2_IRQHandler(void) {
	u8 temp;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)	// 接收寄存器非空
	{
		USART_ClearFlag(USART2, USART_IT_RXNE);	 // USART_FLAG_RXNE        //清除中断标志
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		temp = USART_ReceiveData(USART2);
		blueToothReceive(temp);
	}
}

int tim2Cnt = 0;

void TIM2_IRQHandler(void)	// 1ms
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)	// 溢出中断
	{
		if ((leftboardXscan() && leftboardYscan()) || (rightboardXscan() && rightboardYscan())) {
			tim2Cnt++;
			if (tim2Cnt > 10) {
				tim2Cnt = 0;
			}
		} else
			tim2Cnt--;
		if (tim2Cnt < 0) {
			tim2Cnt = 0;
		}
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	 // 清除中断标志位
}

void TIM3_IRQHandler(void)	// 50ms
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
		if (master.control.begin)
			blueToothSend(2, &master.handle, sizeof(HandleParam));
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

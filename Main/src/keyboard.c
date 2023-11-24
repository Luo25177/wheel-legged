#include "keyboard.h"

void keyboardInit()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //}
}

u8 leftboardXscan(void)
{
	u8				 leftboardx = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	// Y out
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// X in
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2))
		leftboardx |= 0x08;
	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3))
		leftboardx |= 0x04;
	if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		leftboardx |= 0x02;
	if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
		leftboardx |= 0x01;

	return leftboardx;
}

u8 leftboardYscan(void)
{
	u8				 leftboardy = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	// Y in
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// X out
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);

	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
		leftboardy |= 0x04;
	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
		leftboardy |= 0x02;
	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))
		leftboardy |= 0x01;

	return leftboardy;
}

u8 rightboardXscan(void)
{
	u8				 rightboardx = 0;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// X in
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);

	if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
		rightboardx |= 0x08;
	if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
		rightboardx |= 0x04;
	if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
		rightboardx |= 0x02;
	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6))
		rightboardx |= 0x01;

	return rightboardx;
}

u8 rightboardYscan(void)
{
	u8				 rightboardy = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	// Y in
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// X out
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);

	if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))
		rightboardy |= 0x08;
	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9))
		rightboardy |= 0x04;
	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8))
		rightboardy |= 0x02;
	if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))
		rightboardy |= 0x01;

	return rightboardy;
}

void readLeftBoard()
{
	u8 LKEYX = leftboardXscan();
	u8 LKEYY = leftboardYscan();
	switch (LKEYY) {
		case 0x01:
			switch (LKEYX) {
				case 0x01:
					break;
				case 0x02:
					break;
				case 0x04:
					break;
				case 0x08:
					break;
			}
			break;
		case 0x02:
			switch (LKEYX) {
				case 0x01:
					break;
				case 0x02:
					break;
				case 0x04:
					break;
				case 0x08:
					break;
			}
			break;
		case 0x04:
			switch (LKEYX) {
				case 0x01:
					break;
				case 0x02:
					break;
				case 0x04:
					break;
				case 0x08:
					break;
			}
			break;
		default:
			break;
	}
	OSSemPost(beepShowSem);
}

void readRightBoard()
{
	u8 RKEYX = rightboardXscan();
	u8 RKEYY = rightboardYscan();
	switch (RKEYY) {
		case 0x01:
			switch (RKEYX) {
				case 0x01:
					break;
				case 0x02:
					break;
				case 0x04:
					break;
				case 0x08:
					break;
			}
			break;
		case 0x02:
			switch (RKEYX) {
				case 0x01:
					break;
				case 0x02:
					break;
				case 0x04:
					break;
				case 0x08:
					break;
			}
			break;
		case 0x04:
			switch (RKEYX) {
				case 0x01:
					break;

				case 0x02:
					break;
				case 0x04:
					break;
				case 0x08:
					break;
			}
			break;
		default:
			break;
	}
	// TODO: 收到按键消息标志位
	OSSemPost(beepShowSem);
}

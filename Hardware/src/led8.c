#include "led8.h"

const unsigned char ledTable[] = { 0x77, 0x41, 0x3B, 0x6B, 0x4D, 0x6E, 0x7E, 0x43, 0x7F, 0x6F, 0x5F, 0x7C, 0x36, 0x79, 0x3E, 0x1E, 0xFF, 0x00 };

void led8Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	Set_LED8CLK;
	Reset_LED8DATA;
}

void led8Show(unsigned char Data) {
	unsigned char i, val;
	Data %= 17;	 // 防止输入形参超出LED_table数组，造成非法访问
	val		= ledTable[Data];
	for (i = 0; i < 8; ++i) {
		Reset_LED8CLK;
		if (val & 0x01)
			Set_LED8DATA;
		else
			Reset_LED8DATA;
		Set_LED8CLK;
		val >>= 1;
	}
}

#include "delay.h"

void delay_ms(unsigned int t) {
	unsigned int i;
	for (i = 0; i < t; i++) {
		int a = 42000;
		while (a--)
			;
	}
}

/**
 * @brief  延时、微秒
 */
void delay_us(unsigned int t) {
	unsigned int i;
	for (i = 0; i < t; i++) {
		int a = 40;
		while (a--)
			;
	}
}

/**
 * @brief  延时、指令周期数
 */

void delay(u16 t) {
	while (t--)
		;
}

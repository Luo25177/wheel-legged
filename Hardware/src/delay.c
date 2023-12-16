#include "delay.h"

void delay_ms(const unsigned int t) {
	unsigned int i;
	for (i = 0; i < t; ++i) {
		int a = 42000;
		while (a--)
			;
	}
}
void delay_us(const unsigned int t) {
	unsigned int i;
	for (i = 0; i < t; ++i) {
		int a = 40;
		while (a--)
			;
	}
}
void delay(u16 t) {
	while (t--)
		;
}

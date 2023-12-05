#include "vofa.h"

void Vofa_Send_JustFloat(float data1, float data2, float data3, float data4, float data5) {
	u8 SendData[24] = { 0 };
	LF32ToU8(&data1, &SendData[0]);
	LF32ToU8(&data2, &SendData[4]);
	LF32ToU8(&data3, &SendData[8]);
	LF32ToU8(&data4, &SendData[12]);
	LF32ToU8(&data5, &SendData[16]);
	SendData[20] = 0x00;
	SendData[21] = 0x00;
	SendData[22] = 0x80;
	SendData[23] = 0x7f;

	usart2->send(SendData, 24);
}

#include "vofa.h"
void FloatTo4Byte(float* f, u8* buff) {
	u8* p_float;
	p_float		= (unsigned char*) f;	 // µõ½floatµĵؖ·

	*buff			= p_float[0];
	*(++buff) = p_float[1];
	*(++buff) = p_float[2];
	*(++buff) = p_float[3];
}
void Vofa_Send_JustFloat(float data1, float data2, float data3, float data4, float data5) {
	u8 SendData[24] = { 0 };
	FloatTo4Byte(&data1, &SendData[0]);
	FloatTo4Byte(&data2, &SendData[4]);
	FloatTo4Byte(&data3, &SendData[8]);
	FloatTo4Byte(&data4, &SendData[12]);
	FloatTo4Byte(&data5, &SendData[16]);
	SendData[20] = 0x00;
	SendData[21] = 0x00;
	SendData[22] = 0x80;
	SendData[23] = 0x7f;

	usart1->send(SendData, 24);
}

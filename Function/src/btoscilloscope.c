#include "btoscilloscope.h"

#define HEADCHAR 0xA5
#define TAILCHAR 0x5A

void oscilloscope(float* data, u8 size) {
	u8 txmsg[67];	 // 包头包尾校验位 + 数据，最多16个float，但是一般不那么多
	txmsg[0] = HEADCHAR;
	int sz	 = (size << 2) + 1;
	memcpy(&txmsg[1], data, 4 * size);
	int sum = 0;
	for (int i = 1; i < sz; ++i) {
		sum += txmsg[i];
	}
	txmsg[sz++] = sum & 0xff;
	txmsg[sz++] = TAILCHAR;
	usart1->send(txmsg, sz);
}

#include "serialscreen.h"

Serial serial;

void serialReceiveHandler(Serial* serial, u8 data) {
	if (serial->gethead) {
		if (data == SERIALTAILCHAR) {
			serialDealData(serial);
			serial->getmsgsize = 0;
			serial->gethead		 = false;
		}
	} else {
		if (data == SERIALHEADCHAR)
			serial->gethead = true;
	}
}


void serialUpdate(Serial* serial) {
	u8	i			 = 0;
  char str_temp[32];
	u8* TxMesg = usart1->txBuff;
  serial_id_start(serial->faceId);
  serial_text_update(1, (int) robotstate.pitch, "%d");
  serial_text_update(2, (int) robotstate.yaw, "%d");
  serial_text_update(3, (int) robotstate.roll, "%d");
  serial_text_update(4, (int) (robotstate.height * 100), "%d");
  serial_text_update(5, (int) (robotstate.v * 100), "%d");
	serial_end();
	usart1SendSerial(i);
}

void serialDealData(Serial* serial) { }

void serialInit(Serial* serial) {
	serial->faceId		 = 0;
	serial->getmsgsize = 0;
	serial->keyLId		 = 0;
	serial->keyRId		 = 0;
	serial->gethead		 = false;
}

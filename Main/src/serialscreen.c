#include "serialscreen.h"

Serial serial;

void serialReceiveHandler(Serial* serial, u8 data) {
  if(serial->gethead) {
    if(data == SERIALTAILCHAR) {
      serialDealData(serial);
      serial->getmsgsize = 0;
      serial->gethead = false;
    }
  }
  else {
    if(data == SERIALHEADCHAR) 
      serial->gethead = true;
  }
}

char str_temp[32];

void serialUpdate(Serial* serial, u8* TxMesg) {
	u8 i = 0;
	switch (serial->faceId) {
		case 0x01:
		case 0x02:
			serial_id_start(serial->faceId);
			serial_text_update(1, robotstate.pitch, "%f");
			serial_text_update(2, robotstate.yaw, "%f");
			serial_text_update(3, robotstate.roll, "%f");
			serial_text_update(4, robotstate.height, "%f");
			serial_text_update(5, robotstate.v, "%f");
	}
	serial_end();
	usart1SendSerial(i);
}

void serialDealData(Serial* serial) {

}

void serialInit(Serial* serial) {
  serial->faceId = 0;
  serial->getmsgsize = 0;
  serial->keyLId = 0;
  serial->keyRId = 0;
  serial->gethead = false;
}

#include "bluetooth.h"

BlueToothMsg* bluetoothmsg;

//----
// @brief 初始化
// 
//----
void blueToothInit() {
  bluetoothmsg = (BlueToothMsg *) malloc(sizeof(BlueToothMsg));
  bluetoothmsg->gethead = false;
  bluetoothmsg->rxDataSize = 0;
  bluetoothmsg->txDataSize = 0;
}

//----
// @brief 接收信息
// 
// @param data 
//----
void blueToothReceive(u8 data) {
  // 接收到头但是没接到尾 因为接收到尾之后会直接清空flag的
  if(bluetoothmsg->gethead) {
    bluetoothmsg->tail[0] = bluetoothmsg->tail[1];
    bluetoothmsg->tail[1] = data;
    if(bluetoothmsg->tail[0] == TAILCHAR1 && bluetoothmsg->tail[1] == TAILCHAR2) {
      blueToothDeal();
      bluetoothmsg->gethead = false;
      bluetoothmsg->rxDataSize = 0;
    }
    else
      bluetoothmsg->rxData[bluetoothmsg->rxDataSize++] = data;
  }
  // 没接收到头
  else {
    bluetoothmsg->head[0] = bluetoothmsg->head[1];
    bluetoothmsg->head[1] = data;
    if(bluetoothmsg->head[0] == HEADCHAR1 && bluetoothmsg->head[1] == HEADCHAR2) {
      bluetoothmsg->gethead = true;
    }
  }
}

//----
// @brief 接收数据处理
// 
//----
void blueToothDeal() {
  switch(bluetoothmsg->rxData[0]) {
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
  }
}

//----
// @brief 发送信息
// 
// @param id 
// @param data 
// @param size 
//----
void blueToothSend(u8 id, void* data, u8 size) {

}

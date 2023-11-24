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
  bluetoothmsg->txData[0] = HEADCHAR1;
  bluetoothmsg->txData[1] = HEADCHAR2;
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
      memcpy(&master.control, &bluetoothmsg->rxData[1], sizeof(ControlParam));
      break;
    case 2:
      memcpy(&master.handle, &bluetoothmsg->rxData[1], sizeof(HandleParam));
      break;
    case 3:
      memcpy(&robotstate, &bluetoothmsg->rxData[1], sizeof(RobotState));
      break;
    default:
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
  bluetoothmsg->txData[2] = id;
  memcpy(&bluetoothmsg->txData[3], data, size);
  bluetoothmsg->txData[size + 3] = TAILCHAR1;
  bluetoothmsg->txData[size + 4] = TAILCHAR2;
  // size + 5 = head1 + head2 + id + size + tail1 + tail2
  usart1->send(bluetoothmsg->txData, size + 5);
}


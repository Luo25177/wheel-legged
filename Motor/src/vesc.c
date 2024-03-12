#include "vesc.h"

typedef enum { VESCDUTY = 4 } VESCMode;
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS,
  CAN_PACKET_SET_CURRENT_REL,
  CAN_PACKET_SET_CURRENT_BRAKE_REL,
  CAN_PACKET_SET_CURRENT_HANDBRAKE,
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
  CAN_PACKET_STATUS_2,
  CAN_PACKET_STATUS_3,
  CAN_PACKET_STATUS_4,
  CAN_PACKET_PING,
  CAN_PACKET_PONG,
  CAN_PACKET_DETECT_APPLY_ALL_FOC,
  CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
  CAN_PACKET_CONF_CURRENT_LIMITS,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
  CAN_PACKET_CONF_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
  CAN_PACKET_CONF_FOC_ERPMS,
  CAN_PACKET_CONF_STORE_FOC_ERPMS,
  CAN_PACKET_STATUS_5
} CAN_PACKET_ID_Enum;

void VESCInit(VESC* vesc, u8 id) {
  for (int i = 0; i < 4; ++i) {
    vesc[i].monitor.stuckRealse = true;
    vesc[i].monitor.mode        = POSITION;
    vesc[i].monitor.enable      = false;
    vesc[i].set.angleDeg        = 0;
    vesc[i].set.angleRad        = 0;
    vesc[i].set.velocity        = 0;
    vesc[i].set.current         = 0;
    vesc[i].set.torque          = 0;
    vesc[i].param.duty          = 0.1;
    vesc[i].id                  = id++;
    // TODO: 电机参数
    vesc[i].param.polepairs     = VESCU10POLEPAIR;
  }
}

void VESCCommunicate(VESC* vesc) {
  for (int i = 0; i < 4; ++i) {
    CanTxMsg txmsg;
    txmsg.DLC = 4;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.IDE = CAN_Id_Extended;
    switch (vesc[i].monitor.mode) {
      case POSITION:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_POS << 8);
        int pos     = vesc[i].set.angleDeg * 1e6;
        BS32ToU8(&pos, txmsg.Data);
        break;
      case SPEED:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_RPM << 8);
        int speed   = (int) (vesc[i].set.velocity * vesc[i].param.polepairs);
        BS32ToU8(&speed, txmsg.Data);
        break;
      case TORQUE:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_CURRENT << 8);
        int current = (int) (vesc[i].set.current * 1000);
        BS32ToU8(&current, txmsg.Data);
        break;
      case VESCDUTY:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_DUTY << 8);
        int duty    = (int) (vesc[i].param.duty * 100000);
        BS32ToU8(&duty, txmsg.Data);
        break;
      default:
        break;
    }
    can1Txmsg->push(can1Txmsg, txmsg);
  }
}

void VESCMonitor(VESC* vesc) {
  for (int i = 0; i < 4; ++i) {
    ++vesc[i].monitor.timeOutCnt;
    if (vesc[i].monitor.timeOutCnt >= 10)
      vesc[i].monitor.timeOut = true;
  }
}

void VESCReceiveHandle(VESC* vesc, CanRxMsg msg) {
  u8 id = msg.ExtId & 0xff - 1;
  if ((msg.ExtId >> 8) == CAN_PACKET_STATUS) {
    BU8ToF32(msg.Data, &vesc[id].real.velocity);
    vesc[id].real.velocity /= vesc[id].param.polepairs;
    s16 temp                = 0;
    BU8ToS16(&msg.Data[4], &temp);
    vesc[id].real.current = (float) (temp / 10);
    BU8ToS16(&msg.Data[6], &temp);
    float angle = temp / 10;
    if (angle - vesc[id].lastangle > 180)
      vesc[id].n--;
    else if (angle - vesc[id].lastangle < 180)
      vesc[id].n++;
    vesc[id].real.angleDeg = angle + vesc[id].n * 360;
    vesc[id].lastangle     = angle;
  }
}

void VESCRun(VESC* vesc) {
  for (int i = 0; i < 4; ++i) {
    CanTxMsg txmsg;
    txmsg.DLC = 4;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.IDE = CAN_Id_Extended;
    switch (vesc[i].monitor.mode) {
      case POSITION:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_POS << 8);
        int pos     = vesc[i].set.angleDeg * 1e6;
        BS32ToU8(&pos, txmsg.Data);
        break;
      case SPEED:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_RPM << 8);
        int speed   = (int) (vesc[i].set.velocity * vesc[i].param.polepairs);
        BS32ToU8(&speed, txmsg.Data);
        break;
      case TORQUE:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_CURRENT << 8);
        int current = (int) (vesc[i].set.current * 1000);
        BS32ToU8(&current, txmsg.Data);
        break;
      case VESCDUTY:
        txmsg.ExtId = 0xf0000000 | vesc[i].id | ((uint32_t) CAN_PACKET_SET_DUTY << 8);
        int duty    = (int) (vesc[i].param.duty * 100000);
        BS32ToU8(&duty, txmsg.Data);
        break;
      default:
        break;
    }
    can1Txmsg->push(can1Txmsg, txmsg);
  }
}

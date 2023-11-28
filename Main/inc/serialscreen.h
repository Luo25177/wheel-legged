//----
// @file serial.h
// @author mask <beloved25177@126.com>
// @brief
// @version 1.0
// @date 2023-11-24
//
// @copyright Copyright (c) 2023
//
//----
#pragma once

#include "masterparam.h"
#include "stm32f4xx.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>

#define MAXSERIALLEN	 100
#define SERIALHEADCHAR 0xEE
#define SERIALTAILCHAR 0xFF

#define serial_text_update(sub_id, str, type) \
	{                                           \
		TxMesg[i++] = 0x00;                       \
		TxMesg[i++] = (u8) sub_id;                \
		TxMesg[i++] = 0x00;                       \
		sprintf(str_temp, type, str);             \
		TxMesg[i++] = strlen(str_temp);           \
		strcpy((char*) (&TxMesg[i]), str_temp);   \
		i += strlen(str_temp);                    \
	}

// update button control part
#define serial_button_update(sub_id, status) \
	{                                          \
		TxMesg[i++] = 0x00;                      \
		TxMesg[i++] = (u8) sub_id;               \
		TxMesg[i++] = 0x00;                      \
		TxMesg[i++] = 1;                         \
		TxMesg[i++] = status;                    \
	}

// serial start of Bulk updating & set screen id
#define serial_id_start(prio_id) \
	{                              \
		TxMesg[i++] = 0xee;          \
		TxMesg[i++] = 0xb1;          \
		TxMesg[i++] = 0x12;          \
		TxMesg[i++] = 0;             \
		TxMesg[i++] = prio_id;       \
	}

// end of sending
#define serial_end()    \
	{                     \
		TxMesg[i++] = 0xff; \
		TxMesg[i++] = 0xfc; \
		TxMesg[i++] = 0xff; \
		TxMesg[i++] = 0xff; \
	}

// update slider control part
#define serial_slider_update(sub_id, num) \
	{                                       \
		TxMesg[i++] = 0x00;                   \
		TxMesg[i++] = (u8) sub_id;            \
		TxMesg[i++] = 0x00;                   \
		TxMesg[i++] = 0x04;                   \
		TxMesg[i++] = (u8) (num >> 24);       \
		TxMesg[i++] = (u8) (num >> 16);       \
		TxMesg[i++] = (u8) (num >> 8);        \
		TxMesg[i++] = (u8) num;               \
	}

#define serial_selection_update(screen_id, sub_id, num) \
	{                                                     \
		TxMesg[i++] = 0xee;                                 \
		TxMesg[i++] = 0xb1;                                 \
		TxMesg[i++] = 0x10;                                 \
		TxMesg[i++] = 0x00;                                 \
		TxMesg[i++] = screen_id;                            \
		TxMesg[i++] = 0x00;                                 \
		TxMesg[i++] = sub_id;                               \
		TxMesg[i++] = num;                                  \
	}

#define serial_display_hide(screen_id, sub_id, state) \
	{                                                   \
		serial_send_buf[i++] = 0xee;                      \
		serial_send_buf[i++] = 0xb1;                      \
		serial_send_buf[i++] = 0x03;                      \
		serial_send_buf[i++] = 0x00;                      \
		serial_send_buf[i++] = screen_id;                 \
		serial_send_buf[i++] = 0x00;                      \
		serial_send_buf[i++] = sub_id;                    \
		serial_send_buf[i++] = state;                     \
	}

#define serial_display_icon(screen_id, sub_id) \
	{                                            \
		TxMesg[i++] = 0xee;                        \
		TxMesg[i++] = 0xb1;                        \
		TxMesg[i++] = 0x24;                        \
		TxMesg[i++] = 0x00;                        \
		TxMesg[i++] = screen_id;                   \
		TxMesg[i++] = 0x00;                        \
		TxMesg[i++] = sub_id;                      \
		TxMesg[i++] = 0xff;                        \
		TxMesg[i++] = 0xfc;                        \
		TxMesg[i++] = 0xff;                        \
		TxMesg[i++] = 0xff;                        \
	}

typedef struct {
	u8 faceId;
	u8 controlId;
	u8 keyLId;
	u8 keyRId;

	u8	 rxdata[MAXSERIALLEN];
	u8	 getmsgsize;
	bool gethead;
} Serial;

extern Serial serial;

void serialReceiveHandler(Serial* serial, u8 data);
void serialUpdate(Serial* serial);
void serialDealData(Serial* serial);
void serialInit(Serial* serial);

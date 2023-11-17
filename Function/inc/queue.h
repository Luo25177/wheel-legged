//----
// @file queue.h
// @author mask (beloved25177@126.com)
// @brief 
// @version 1.0
// @date 2023-11-11
// 
// @copyright Copyright (c) 2023
// 
//----
#pragma once

#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stm32f4xx_can.h"

#define QUEUEMAXSIZE 10

typedef struct CanTxQueue {
	u8 tail, head;
	CanTxMsg data[QUEUEMAXSIZE];
	bool (*isFull)(struct CanTxQueue* queue);
	bool (*isEmpty)(struct CanTxQueue* queue);
	bool (*enqueue)(struct CanTxQueue* queue, void* data);
	bool (*dequeue)(struct CanTxQueue* queue, void* data);
} CanTxQueue;

typedef struct CanRxQueue {
	int	tail, head;
	CanRxMsg data[QUEUEMAXSIZE];
	bool (*isFull)(struct CanRxQueue* queue);
	bool (*isEmpty)(struct CanRxQueue* queue);
	bool (*enqueue)(struct CanRxQueue* queue, void* data);
	bool (*dequeue)(struct CanRxQueue* queue, void* data);
} CanRxQueue;

CanRxQueue* newCanRxQueue();
CanTxQueue* newCanTxQueue();

extern CanRxQueue* can1RxMsg;
extern CanTxQueue* can1TxMsg;
extern CanRxQueue* can2RxMsg;
extern CanTxQueue* can2TxMsg;


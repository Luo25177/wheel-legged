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

#include "stdbool.h"
#include "stdlib.h"
#include "stm32f4xx_can.h"
#include "string.h"

#define QUEUEMAXSIZE 10
#define queue(T) queue_##T
#define newqueue(T) newqueue_##T
#define deletequeue(T) deletequeue_##T
#define Queue(T)                                   \
  typedef struct queue_##T {                       \
    T *val;                                        \
    int head;                                      \
    int tail;                                      \
    int size;                                      \
    int (*maxsize)(struct queue_##T * queue);      \
    int (*getsize)(struct queue_##T * queue);      \
    int (*isempty)(struct queue_##T * queue);      \
    int (*isfull)(struct queue_##T * queue);       \
    void (*push)(struct queue_##T * queue, T val); \
    bool (*pop)(struct queue_##T * queue, T *val); \
    void (*clear)(struct queue_##T * queue);       \
  } queue_##T;
#define ExternNewQueue(T) extern queue_##T *newqueue_##T(int size);
#define ExternDeleteQueue(T) extern void deletequeue_##T(queue_##T *queue);
Queue(CanRxMsg);
Queue(CanTxMsg);
ExternNewQueue(CanRxMsg);
ExternNewQueue(CanTxMsg);
ExternDeleteQueue(CanRxMsg);
ExternDeleteQueue(CanTxMsg);

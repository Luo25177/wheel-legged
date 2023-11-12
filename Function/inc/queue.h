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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define Queue(T) typedef struct {\
  T *val;\
  int head;\
  int tail;\
  int size;\
  int (*maxsize)(struct queue_##T* queue);\
  int (*getsize)(struct queue_##T* queue);\
  int (*isempty)(struct queue_##T* queue);\
  int (*isfull)(struct queue_##T* queue);\
  void (*push)(struct queue_##T* queue, T val);\
  void (*pop)(struct queue_##T* queue);\
  T (*top)(struct queue_##T* queue);\
  void (*clear)(struct queue_##T *queue);\
} queue_##T;
#define queue(T) queue_##T
#define Maxsize(T) int maxsize_##T(queue_##T *queue){return queue->size;}
#define Getsize(T) int getsize_##T(queue_##T *queue){\
  int size = queue->tail - queue->head;\
  while(size < 0)size += queue->size;\
  while(size > queue->size)size -= queue->size;\
  return size;\
}
#define Isempty(T) int isempty_##T(queue_##T *queue){\
  return queue->head == queue->tail;\
}
#define Isfull(T) int isfull_##T(queue_##T *queue){\
  return queue->tail == (queue->head - 1 >= 0? queue->head - 1 : queue->head - 1 + queue->size);\
}
#define Push(T) void push_##T(queue_##T *queue, T val){\
  if(queue->tail == (queue->head - 1 >= 0? queue->head - 1 : queue->head - 1 + queue->size))return;\
  queue->val[queue->tail] = val;\
  queue->tail += 1;\
  queue->tail %= queue->size;\
}
#define Pop(T) void pop_##T(queue_##T *queue){\
  if(queue->head == queue->tail){\
    printf("this queue is empty");\
    return;\
  }\
  queue->head++;\
  queue->head %= queue->size;\
}
#define Top(T) T top_##T(queue_##T *queue){\
  if(queue->head == queue->tail){\
    printf("this queue is empty");\
    return (T)0;\
  }\
  return queue->val[queue->head];\
}
#define Clear(T) void clear_##T(queue_##T *queue){\
  queue->head = 0;\
  queue->tail = 0;\
}
#define NewQueue(T) queue_##T *newqueue_##T(int size){\
  queue_##T *queue = (queue_##T *) malloc(sizeof(queue_##T));\
  queue->val = (T *) malloc(sizeof(T) * size);\
  queue->size = size;\
  queue->head = 0;\
  queue->tail = 0;\
  queue->maxsize = maxsize_##T;\
  queue->getsize = getsize_##T;\
  queue->isempty = isempty_##T;\
  queue->isfull = isfull_##T;\
  queue->push = push_##T;\
  queue->pop = pop_##T;\
  queue->top = top_##T;\
  queue->clear = clear_##T;\
  return queue;\
}
#define newqueue(T) newqueue_##T
#define DeleteQueue(T) void deletequeue_##T(queue_##T* queue){\
  free(queue->val);\
  free(queue);\
}
#define deletequeue(T) deletequeue_##T
#define DEFINEQUEUE(T) Queue(T)\
  Maxsize(T)\
  Getsize(T)\
  Isempty(T)\
  Isfull(T)\
  Push(T)\
  Pop(T)\
  Top(T)\
  Clear(T)\
  NewQueue(T)\
  DeleteQueue(T)
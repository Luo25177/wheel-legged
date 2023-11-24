#include "queue.h"

#define Maxsize(T) int maxsize_##T(queue_##T *queue) {return queue->size;}
#define Getsize(T) int getsize_##T(queue_##T *queue) {\
  int size = queue->tail - queue->head;\
  while(size < 0)size += queue->size;\
  while(size > queue->size)size -= queue->size;\
  return size;\
}
#define Isempty(T) int isempty_##T(queue_##T *queue) {\
  return queue->head == queue->tail;\
}
#define Isfull(T) int isfull_##T(queue_##T *queue) {\
  return queue->tail == (queue->head - 1 >= 0? queue->head - 1 : queue->head - 1 + queue->size);\
}
#define Push(T) void push_##T(queue_##T *queue, T val) {\
  if(queue->tail == (queue->head - 1 >= 0? queue->head - 1 : queue->head - 1 + queue->size))return;\
  *(queue->val + queue->tail) = val;\
  queue->tail += 1;\
  queue->tail %= queue->size;\
}
#define Pop(T) bool pop_##T(queue_##T *queue, T *val) {\
  if(queue->head == queue->tail) \
    return false;\
  *val = *(queue->val + queue->head);\
  queue->head++;\
  queue->head %= queue->size;\
	return true;\
}
#define Clear(T) void clear_##T(queue_##T *queue) {\
  queue->head = 0;\
  queue->tail = 0;\
}
#define NewQueue(T) queue_##T *newqueue_##T(int size) {\
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
  queue->clear = clear_##T;\
  return queue;\
}
#define DeleteQueue(T) void deletequeue_##T(queue_##T* queue){\
  free(queue->val);\
  free(queue);\
}
#define DEFINEQUEUE(T) Maxsize(T)\
  Getsize(T)\
  Isempty(T)\
  Isfull(T)\
  Push(T)\
  Pop(T)\
  Clear(T)\
  NewQueue(T)\
  DeleteQueue(T)

DEFINEQUEUE(CanTxMsg);
DEFINEQUEUE(CanRxMsg);

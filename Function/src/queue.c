#include "queue.h"

bool canTxQueueIsFull(CanTxQueue* queue)
{
	if (queue->head == (queue->tail + 1) % QUEUEMAXSIZE)
		return true;
	return false;
}
bool canRxQueueIsFull(CanRxQueue* queue)
{
	if (queue->head == (queue->tail + 1) % QUEUEMAXSIZE)
		return true;
	return false;
}

bool canTxQueueIsEmpty(CanTxQueue* queue)
{
	if (queue->tail == queue->head)
		return true;
	return false;
}
bool canRxQueueIsEmpty(CanRxQueue* queue)
{
	if (queue->tail == queue->head)
		return true;
	return false;
}

bool canTxEnqueue(CanTxQueue* queue, void* data)
{
	if (queue->isFull(queue))
		return false;
	memcpy(&(queue->data[queue->tail]), data, sizeof(CanTxMsg));
	queue->tail++;
	queue->tail %= QUEUEMAXSIZE;
	return true;
}
bool canRxEnqueue(CanRxQueue* queue, void* data)
{
	if (queue->isFull(queue))
		return false;
	memcpy(&(queue->data[queue->tail]), data, sizeof(CanRxMsg));
	queue->tail++;
	queue->tail %= QUEUEMAXSIZE;
	return true;
}

bool canTxDequeue(CanTxQueue* queue, void* data)
{
	if (queue->isEmpty(queue))
		return false;
	memcpy(data, &(queue->data[queue->head]), sizeof(CanTxMsg));
	queue->head++;
	queue->head %= QUEUEMAXSIZE;
	return true;
}
bool canRxDequeue(CanRxQueue* queue, void* data)
{
	if (queue->isEmpty(queue))
		return false;
	memcpy(data, &(queue->data[queue->head]), sizeof(CanRxMsg));
	queue->head++;
	queue->head %= QUEUEMAXSIZE;
	return true;
}

CanRxQueue* newCanRxQueue()
{
	CanRxQueue* queue = (CanRxQueue*) malloc(sizeof(CanRxQueue));
	if (!queue)
		return NULL;
	queue->isEmpty = canRxQueueIsEmpty;
	queue->enqueue = canRxEnqueue;
	queue->dequeue = canRxDequeue;
	queue->isFull  = canRxQueueIsFull;
	queue->head	   = 0;
	queue->tail	   = 0;
	return queue;
}
CanTxQueue* newCanTxQueue()
{
	CanTxQueue* queue = (CanTxQueue*) malloc(sizeof(CanTxQueue));
	if (!queue)
		return NULL;
	queue->isEmpty = canTxQueueIsEmpty;
	queue->enqueue = canTxEnqueue;
	queue->dequeue = canTxDequeue;
	queue->isFull  = canTxQueueIsFull;
	queue->head	   = 0;
	queue->tail	   = 0;
	return queue;
}
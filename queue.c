#include "queue.h"

Queue_Error errq = E_NQE;

void initialize(Queue* q) {
	q->first = -1;
	q->last = -1;
	q->space = QUEUE_SIZE;
}

void queue(Queue* q, byte b) {
	if(isFull(q)) {
		errq = E_FL;
		return;
	}
	q->last = (q->last + 1)%QUEUE_SIZE;
	*(q->buf + q->last) = b;
	if(isEmpty(q)) q->first = (q->first + 1)%QUEUE_SIZE;
	q->space--;
	
	if(q->space == 2) {
		errq = E_AFL;
	} else {
		errq = E_NQE;
	}
	
	return;
}

byte dequeue(Queue* q) {
	if(isEmpty(q)) {
		errq = E_EM;
		return 0;
	}
	byte b = *(q->buf + q->first);
	if(q->first == q->last) {
		q->first = -1;
		q->last = -1;
	}else {
		q->first = (q->first + 1)%QUEUE_SIZE;
	}
	q->space++;
	
	if(q->space == QUEUE_SIZE - 2) {
		errq = E_AEM;
	} else {
		errq = E_NQE;
	}
	
	return b;
}

bool isFull(Queue* q) {
	//return ((q->last + 1)%QUEUE_SIZE == q->first);
	return (q->space == 0);
}

bool isEmpty(Queue* q) {
	//return (q->first == -1 && q->last == -1);
	return (q->space == QUEUE_SIZE);
}
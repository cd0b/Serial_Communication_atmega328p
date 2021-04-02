#ifndef _QUEUE_H
#define _QUEUE_H
#include "bool.h"

#define QUEUE_SIZE 16

typedef enum {E_NQE, E_FL, E_EM, E_AFL, E_AEM} Queue_Error;
extern Queue_Error errq;

typedef unsigned char byte;

typedef struct Queue {

	byte buf[QUEUE_SIZE];
	byte first;
	byte last;
	byte space;
	
}Queue;

void initialize(Queue*);
void queue(Queue*, byte);
byte dequeue(Queue*);
bool isFull(Queue*);
bool isEmpty(Queue*);

#endif
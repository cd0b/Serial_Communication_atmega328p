#ifndef _SERIAL_H
#define _SERIAL_H

#include "queue.h"

#define clock_frequency 1000000

typedef enum {E_NSE, E_RBE, E_SBF} Serial_Error;
extern Serial_Error errs;

void serial(int, byte, int);
void serial_write(byte);
byte serial_read();


#endif
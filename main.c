
#include <avr/io.h>
#include "serial.h"



int main(void) {
	
	serial(9600, (byte)'C', 0);
	
	
	serial_write(0xf0);
	while(errs != E_NSE) serial_write(0xf0);
	
	byte b = serial_read();
	while(errs != E_NSE) b = serial_read();
	
	DDRD = 0xff;
	PORTD = b;
	
	while(1);
	
}
#include "serial.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/*
	pin set and clear and read
*/
#define rx ((bool)((*(settings.pin) >> settings.rx_pin) & 0x01))
#define lsm ((bool)((*(settings.pin) >> settings.lsm_pin) & 0x01))
#define liws ((bool)((*(settings.pin) >> settings.liws_pin) & 0x01))
#define set_tx \
			{*(settings.port) |= (1 << settings.tx_pin);}
#define clr_tx \
			{*(settings.port) &= ~(1 << settings.tx_pin);}
#define set_sm \
			{*(settings.port) |= (1 << settings.sm_pin);}
#define clr_sm \
			{*(settings.port) &= ~(1 << settings.sm_pin);}
#define set_iws \
			{*(settings.port) |= (1 << settings.iws_pin);}
#define clr_iws \
			{*(settings.port) &= ~(1 << settings.iws_pin);}



/*
	set clocks
*/
#define set_write_clock \
			{ \
				OCR0A = settings.interrupt_clock_count; \
				TCCR0A |= (1 << WGM01); \
				TCCR0B |= (1 << CS00); \
				sei(); \
			}
#define set_read_clock \
			{ \
				OCR0B = settings.interrupt_clock_count-20; \
				TCCR0A |= (1 << WGM01); \
				TCCR0B |= (1 << CS00); \
				sei(); \
			}
#define turn_read_clock \
			{ \
				TIMSK0 &= ~(1 << OCIE0A); \
				TIMSK0 |= (1 << OCIE0B); \
			}
#define turn_write_clock \
			{ \
				TIMSK0 &= ~(1 << OCIE0B); \
				TIMSK0 |= (1 << OCIE0A); \
			}
			


Serial_Error errs = E_NSE;

typedef byte* reg;
typedef struct {
	
	int interrupt_clock_count;
	byte rx_pin;
	byte tx_pin;
	byte sm_pin;
	byte lsm_pin;
	byte iws_pin;
	byte liws_pin;
	reg port;
	reg pin;
		
} Settings ;

typedef struct {
	
	Queue internal_send_queue;
	Queue internal_recv_queue;
	
} Buffers;


Settings settings;
Buffers buffers;

void set_ports(byte, int);


void serial(int baud_rate, byte port, int offset) {
	
	initialize(&buffers.internal_recv_queue);
	initialize(&buffers.internal_send_queue);
	
	settings.interrupt_clock_count = (int)(clock_frequency/baud_rate);
	
	set_ports(port, offset);
	set_sm;
	
	set_read_clock;
	set_write_clock;
	turn_write_clock;
	
	return;
	
}


void set_ports(byte port, int offset) {
	switch(port) {
		case 'C':
			settings.rx_pin = offset%7;
			settings.tx_pin = (offset+1)%7;
			settings.sm_pin = (offset+2)%7;
			settings.lsm_pin = (offset+3)%7;
			settings.iws_pin = (offset+4)%7;
			settings.liws_pin = (offset+5)%7;
			DDRC = (1<<settings.tx_pin) | 
					(1 << settings.sm_pin) |  
					(1 << settings.iws_pin);
			PORTC = (1 << settings.rx_pin) | // internal pull up
					(1 << settings.lsm_pin) |
					(1 << settings.liws_pin);
			settings.port = 0x28; // PORTC
			settings.pin = 0x26; // PINC
			break;
		case 'B':
			settings.rx_pin = offset%8;
			settings.tx_pin = (offset+1)%8;
			settings.sm_pin = (offset+2)%8;
			settings.lsm_pin = (offset+3)%8;
			settings.iws_pin = (offset+4)%8;
			settings.liws_pin = (offset+5)%8;
			DDRB = (1 << settings.tx_pin) |
					(1 << settings.sm_pin) |
					(1 << settings.iws_pin);
			PORTB = (1 << settings.rx_pin) | // internal pull up
					(1 << settings.lsm_pin) |
					(1 << settings.liws_pin);
			settings.port = 0x25; // PORTB
			settings.pin = 0x23; // PINB
			break;
		case 'D':
			settings.rx_pin = offset%8;
			settings.tx_pin = (offset+1)%8;
			settings.sm_pin = (offset+2)%8;
			settings.lsm_pin = (offset+3)%8;
			settings.iws_pin = (offset+4)%8;
			settings.liws_pin = (offset+5)%8;
			DDRD = (1<<settings.tx_pin) |
					(1 << settings.sm_pin) |
					(1 << settings.iws_pin);
			PORTD = (1 << settings.rx_pin) | // internal pull up
					(1 << settings.lsm_pin) |
					(1 << settings.liws_pin);
			settings.port = 0x2B; // PORTD
			settings.pin = 0x29; // PIND
	}
	MCUCR &= ~(1 << PUD); // internal pull up
	return;
}

// write
int snd_bit = -1;
byte snd_byte = 0;
ISR(TIMER0_COMPA_vect) {
	
	if(lsm) {
		switch(snd_bit) {
			case -1:
				snd_byte = dequeue(&buffers.internal_send_queue);
				if(errq == E_EM) { clr_iws; clr_tx; break; }
				set_iws;
				set_tx; // start bit
				++snd_bit;
				break;
			case 8:
				set_tx;
				snd_bit = -1;
				snd_byte = 0;
				break;
			default:
				if((snd_byte >> snd_bit) & 0x01) { set_tx; } else { clr_tx; }
				++snd_bit;
		}
	} else {
		clr_tx;
	}
	
		
	turn_read_clock;
}

// read
int rcv_bit = -1;
byte rcv_byte = 0;
ISR(TIMER0_COMPB_vect) {
	
	if(liws) {
		switch(rcv_bit) {
			case -1:
				if(rx) {
					++rcv_bit;	
				}
				break;
			case 8:
				// control parity
				queue(&buffers.internal_recv_queue, rcv_byte);
				if(errq == E_AFL) clr_sm;
				rcv_bit = -1;
				rcv_byte = 0;
				break;
			default:
				rcv_byte |= rx << rcv_bit;
				++rcv_bit;
		}
	}
	
	turn_write_clock;
}





void serial_write(byte b) {
	cli();
	queue(&buffers.internal_send_queue, b);
	
	if(errq != E_FL) {
		errs = E_NSE;
	} else {
		errs = E_SBF;
	}
	sei();
	return;
}

byte serial_read() {
	cli();
	byte b = dequeue(&buffers.internal_recv_queue);
	
	if(errq == E_AEM) set_sm;
	
	if(errq != E_EM) {
		errs = E_NSE;
	} else { 
		errs = E_RBE; 
	}
	sei();
	return b;
	
}
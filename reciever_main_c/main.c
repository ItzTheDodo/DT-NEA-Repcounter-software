
#define F_CPU 1000000L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "STDIO_UART.h"

#include "nrf24l01.h"
#include "nrf24l01-mnemonics.h"
#include "spi.h"
#include "nrf24l01.c"
#include "spi.c"
#include "STDIO_UART.c"

#ifndef BAUD
#define BAUD 9600
#endif

uint8_t getDigitalBit(volatile uint8_t * reg, uint8_t pin) {
  return (uint8_t) (*reg & (1 << pin)) >> pin;
}

const int segment_numeral[10] = {
	0b1111110, //0
	0b0110000, //1
	0b1101101, //2
	0b1111001, //3
	0b0110011, //4
	0b1011011, //5
	0b0011111, //6
	0b1110000, //7
	0b1111111, //8
	0b1110011, //9
};

// G1, G2, G3, G4
const int digitPins[] = {
	PD0,
	PD1,
	PD3,
	PD4
};

void showDigit(int number, int digit) {
	PORTD |= (1 << digitPins[digit]);
	PORTC = 0x0;
	int pc_pins = segment_numeral[number] & ~(1);
	int pb_pins = segment_numeral[number] & 0b0000001;
	PORTC = pc_pins;
	PORTB |= pb_pins;
	_delay_ms(50);
	PORTD &= ~(1 << digitPins[digit]);
}

void showNumber(int number) {
	int digit;
	for (digit = 0; digit < 4; digit++) {
		showDigit(number % 10, digit);
		number /= 10;
	}
}

void print_config(void);

volatile bool message_received = false;
volatile bool status_tx = false;
volatile bool status_conn = false;

// bool control vars //
bool mode = false; // true = timer, false=counter
bool ss_timer = false; // true = started, false=stopped

volatile unsigned int timer_count = 0;
volatile unsigned int counter = 0;

int main(void) {
	
	_delay_ms(500);
	
	// PD0-1 = G1-2
	// PD3-4 = G3-4
	// PD5 = dp
	// PD6 = start_timer
	// PD7 = stop_timer
	// PortC = A-G
	// PB7 = Mode_switch
	// PB6 = reset_timer
	
	DDRD |= (1 << DDD0);
	DDRD |= (1 << DDD1);
	DDRD |= (1 << DDD3);
	DDRD |= (1 << DDD4);
	DDRD |= (1 << DDD5);
	DDRD &= ~(1 << DDD6);
	DDRD &= ~(1 << DDD7);
	
	DDRC |= (1 << DDC0);
	DDRC |= (1 << DDC1);
	DDRC |= (1 << DDC2);
	DDRC |= (1 << DDC3);
	DDRC |= (1 << DDC4);
	DDRC |= (1 << DDC5);
	
	DDRB |= (1 << DDB0);
	
	DDRB &= ~(1 << DDB6);
	DDRB &= ~(1 << DDB7);
	
	// setup 1 second timer interrupt
	cli();
	TCCR1B |= (1 << WGM12);
	TIMSK1 |= (1 << OCIE1A);
	OCR1A = 15624;
	TCCR1B |= ((1 << CS10) | (1 << CS11));
	sei();
	
	
	char tx_message[32];
	
	spi_master_init();
	uart_init();
	nrf24_init();
	print_config();
	
	nrf24_start_listening();
	
	while (1) {
		
		// init handshake
		if (message_received && !status_conn) {
			message_received = false;
			char *check = "connected_rc";
			if (nrf24_read_message() == check) {
				status_conn = true;
			}
		}
		
		if (!status_conn) {
			strcpy(tx_message, "connected_rc");
			status_tx = nrf24_send_message(tx_message);
			continue;
		}
		
		if (getDigitalBit(&PORTB, PB7)) {
			mode = !mode;
			showNumber(0);
		}
		
		if (message_received && !mode) {
			char *check = "1";
			if (nrf24_read_message() == check) {
				if (counter > 9999) {
					continue;
				}
				counter += 1;
				showNumber(counter);
			}
		
		}
		
		if (getDigitalBit(&PORTD, PD6) && mode) {
			ss_timer = true;
		}
		if (getDigitalBit(&PORTD, PD7) && mode) {
			ss_timer = false;
		}
		if (getDigitalBit(&PORTB, PD6) && mode) {
			timer_count = 0;
		}
		
	}
	
}

ISR(TIMER1_COMPA_vect) {
	if (mode && ss_timer) {
		timer_count += 1;
		showNumber(timer_count);
	}
}

ISR(INT0_vect) {
	message_received = true;
}

void print_config(void)
{
	uint8_t data;
	printf("Startup successful\n\n nRF24L01+ configured as:\n");
	printf("-------------------------------------------\n");
	nrf24_read(CONFIG,&data,1);
	printf("CONFIG		0x%x\n",data);
	nrf24_read(EN_AA,&data,1);
	printf("EN_AA			0x%x\n",data);
	nrf24_read(EN_RXADDR,&data,1);
	printf("EN_RXADDR		0x%x\n",data);
	nrf24_read(SETUP_RETR,&data,1);
	printf("SETUP_RETR		0x%x\n",data);
	nrf24_read(RF_CH,&data,1);
	printf("RF_CH			0x%x\n",data);
	nrf24_read(RF_SETUP,&data,1);
	printf("RF_SETUP		0x%x\n",data);
	nrf24_read(STATUS,&data,1);
	printf("STATUS		0x%x\n",data);
	nrf24_read(FEATURE,&data,1);
	printf("FEATURE		0x%x\n",data);
	printf("-------------------------------------------\n\n");
}


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

struct AccVal {
	unsigned int Z;
	unsigned int Y;
	unsigned int X;
};

struct PAccData {
	bool threshhold_change;
	bool z_sig; // 1 = +, 0 = -
};

void print_config(void);

struct AccVal past_val = {0, 0, 0};
unsigned int acc_z_threshhold = 1000; // mg
unsigned int acc_y_threshhold = 500; // mg
unsigned int acc_x_threshhold = 500; // mg

unsigned int readAnaloguePin(uint8_t ADC_PIN) {
	unsigned int pin_val = ADC_PIN & 0xF;
	ADMUX &= 0xF0;
	ADMUX |= pin_val;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return (ADCH << 8) & ADCL;  // return 10 bit value containing strength of signal
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct PAccData check_accelorometer(void) {
	//raw values
	unsigned int Z = readAnaloguePin(1);
	unsigned int Y = readAnaloguePin(2);
	unsigned int X = readAnaloguePin(3);
	
	//mili-gs
	long Z_g = map(Z, 0, 1023, -3000, 3000);
	long Y_g = map(Y, 0, 1023, -3000, 3000);
	long X_g = map(X, 0, 1023, -3000, 3000);
	
	// calculate whether change in acceleration
	bool change = (abs(past_val.Z - Z_g) >= acc_z_threshhold && abs(past_val.Y - Y_g) >= acc_y_threshhold && abs(past_val.X - X_g) >= acc_x_threshhold);
	bool sig =  (Z_g > 0);
	past_val.Z = Z_g;
	past_val.Y = Y_g;
	past_val.X = X_g;
	// format for output
	struct PAccData data = {change, sig};
	return data;
}

volatile bool message_received = false;
volatile bool status_conn = false;
volatile bool status_tx = false;

bool has_passed_threshhold = false;

int main (void) {
	
	_delay_ms(500);
	
	// Enable adc pins 0-3
	DIDR0 |= (1 << ADC0D);
	DIDR0 |= (1 << ADC1D);
	DIDR0 |= (1 << ADC2D);
	DIDR0 |= (1 << ADC3D);
	
	//// ADC SETUP ////
	ADCSRA |= (1 << ADEN); // enable the ADC

	// set ADC prescaler to max
	ADCSRA |= (1 << ADPS0);
	ADCSRA |= (1 << ADPS1);
	ADCSRA |= (1 << ADPS2);

	// set voltage referance pins to internal 1.1v + cap
	ADMUX |= (1 << REFS0);
	ADMUX |= (1 << REFS1);
	///////////////////
	
	
	char tx_message[32];
	
	spi_master_init();
	uart_init();
	nrf24_init();
	print_config();
	
	nrf24_start_listening();
	
	while (1) {
		
		if (message_received && !(status_conn)) {  // check connected req & init handshake
			message_received = false;
			char *check = "connected_rc";
			if (nrf24_read_message() == check) {
				strcpy(tx_message, "connected_rc");
				status_tx = nrf24_send_message(tx_message);
				status_conn = status_tx;
			}
		}
		
		if (!status_conn) {
			continue;
		}
		
		// when up: has pass th,
		// when down: has pass th, count rep if not yet
		
		struct PAccData cur_data = check_accelorometer();
		if (cur_data.z_sig) { // up
			has_passed_threshhold = cur_data.threshhold_change;
		} else if (has_passed_threshhold && cur_data.threshhold_change) {
			has_passed_threshhold = false;
			// count rep
			strcpy(tx_message, "1");
			status_tx = nrf24_send_message(tx_message);
			status_conn = status_tx;
		}
		
		
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

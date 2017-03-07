/****************************************************************************
* Firmware for Finger v2 (Electric Field Sensing Board)
*
* Brian Mayton <bmayton@cs.washington.edu>
* (c) Intel Research Seattle, October 2008
*
* Lightly edited by David Michelman, daweim0@gmai.com or davidmic@uw.edu
*
* This file handles the UART for RS232 communication, including buffered
* reads.
****************************************************************************/

#warning "uartlib0.c defined"

#include <stdint.h>
//#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <assert.h>
//#include <util/delay.h>
//#include <stdio.h>
#include "uartLib0.h"


uint8_t rxbuf0[8];
volatile uint8_t rdptr0 = 0;
volatile uint8_t wrptr0 = 0;

void uart_setup0(void) {
	UBRR0H = 0;
	//UBRR0L = 0; // 2mbps (with double speed)
	UBRR0L = 1; // 1000000 bps (with double speed taken into account)
	//UBRR0L = 3; // 500000 bps (with double speed taken into account)
	//UBRR0L = 16; // 115.2 kbps (with double speed taken into account)
	//UBRR0H = 207 >> 8;
	//UBRR0L = 34; // 57600 bps (with double speed taken into account)
	UCSR0A |= (1<<U2X1);   /* double speed */
	UCSR0B = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);
	UCSR0C = (1 << UCSZ11) | (1 << UCSZ10); /* 8 bit */

}

void uart_putchar0(uint8_t c) {
	while( ! (UCSR0A & (1<<UDRE)) );
	UDR0 = c;
}

/* Prints out a null terminated string */
void uart_putstr0(char *str) {
	while (*str)
	uart_putchar0(*str++);
}

void debug_print(char *str) {
	uart_putstr0(str);
}

void uart_putdata0(uint8_t *data, uint8_t count) {
	int i;
	for(i=0; i<count; i++) uart_putchar0(data[i]);
}

char uart_getchar0(void) {   // returns 0xEE00 if no data is present.
	char c = rxbuf0[rdptr0];
	rdptr0 = (rdptr0 + 1) & 7;
	return c;
}
uint8_t uart_hasNext0(void){
	return rdptr0 != wrptr0;
}

void SEND160(uint16_t x){
	uart_putchar0(x >> 8); //only upper byte
	uart_putchar0(x & 0xFF); //only lower byte
	

	/**
	* UART Receive ISR
	*/
}
ISR(USART0_RX_vect) {
	rxbuf0[wrptr0] = UDR0;
	wrptr0 = (wrptr0 + 1) & 7; //& 7 forces wrptr to wrap
	if(wrptr0 == rdptr0)       //wrptr is data write head
	rdptr0 = (rdptr0 + 1) & 7; // rdprt is data read head
}

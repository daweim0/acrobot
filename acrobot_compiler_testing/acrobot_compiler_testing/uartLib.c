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

#ifndef uartlib1_included
#define uartlib1_included

#include "uartLib.h"

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <assert.h>
#include <util/delay.h>
#include <stdio.h>

void uart_setup() {
	UBRR1H = 0;
	//UBRR1L = 0; // 2mbps (with double speed)
	//UBRR1L = 3; // 500000 bps (with double speed taken into account)
	UBRR1L = 16; // 115.2 kbps (with double speed taken into account)
	//UBRR1H = 207 >> 8;
	//UBRR1L = 34; // 57600 bps (with double speed taken into account)
	UCSR1A |= (1<<U2X1);   /* double speed */
	UCSR1B = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); /* 8 bit */

}

void uart_putchar(uint8_t c) {
	while( ! (UCSR1A & (1<<UDRE)) );
	UDR1 = c;
}

/* Prints out a null terminated string */
void uart_putstr(char *str) {
	while (*str)
	uart_putchar(*str++);
}

void uart_putdata(uint8_t *data, uint8_t count) {
	int i;
	for(i=0; i<count; i++) uart_putchar(data[i]);
}

char uart_getchar() {   //returns 0xEE00 if no data is present.
	char c = rxbuf[rdptr];
	rdptr = (rdptr + 1) & 7;
	return c;
}

uint8_t uart_hasNext(){
	return rdptr != wrptr;
}

void SEND16(uint16_t x){
	uart_putchar(x >> 8); //only upper byte
	uart_putchar(x & 0xFF); //only lower byte
}

	/**
	* UART Receive ISR
	*/

ISR(USART1_RX_vect) {
	rxbuf[wrptr] = UDR1;
	wrptr = (wrptr + 1) & 7; //& 7 forces wrptr to wrap
	if(wrptr == rdptr)       //wrptr is data write head
	rdptr = (rdptr + 1) & 7; // rdprt is data read head
}

#endif
/****************************************************************************
 * Firmware for Finger v2 (Electric Field Sensing Board)
 *
 * Brian Mayton <bmayton@cs.washington.edu>
 * (c) Intel Research Seattle, October 2008
 *
 * This file handles the UART for RS232 communication, including buffered
 * reads.
 ****************************************************************************/

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// F_CPU defined in Makefile

// UART buffers
uint8_t rxbuf[8];
volatile uint8_t rdptr = 0;
volatile uint8_t wrptr = 0;

/*
 * Set up the UART
 */
void uart_setup() {
    UBRR1H = 0;
    UBRR1L = 8;
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

int16_t uart_getchar() {
    if(rdptr == wrptr)
        return -1;
    
    char c = rxbuf[rdptr];
    rdptr = (rdptr + 1) & 7;
    return c;
}

/**
 * UART Receive ISR
 */
ISR(USART1_RX_vect) {
    rxbuf[wrptr] = UDR1;
    wrptr = (wrptr + 1) & 7;
    if(wrptr == rdptr)
        rdptr = (rdptr + 1) & 7;
}

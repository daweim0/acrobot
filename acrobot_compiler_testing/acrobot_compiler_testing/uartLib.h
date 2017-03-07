
#ifndef uartLib_h
#define uartLib_h

#include <stdint.h>

uint8_t rxbuf[8];
volatile uint8_t rdptr = 0;
volatile uint8_t wrptr = 0;

void setPower(int8_t);
void init_all(void);
void SEND16(uint16_t value);
void uart_setup();
void uart_putchar(uint8_t c);
void uart_putstr(char *str);
void uart_putdata(uint8_t *data, uint8_t count);
char uart_getchar(void);
uint8_t uart_hasNext(void);

#include "uartLib.c"

#endif
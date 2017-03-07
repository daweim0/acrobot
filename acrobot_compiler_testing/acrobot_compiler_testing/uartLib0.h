#ifndef uartLib0_h
#define uartLib0_h


void setPower0(int8_t);
void init_all0(void);
void SEND160(uint16_t value);
void uart_setup0(void);
void uart_putchar0(uint8_t c);
void uart_putstr0(char *str);
void uart_putdata0(uint8_t *data, uint8_t count);
char uart_getchar0(void);
uint8_t uart_hasNext0(void);
void debug_print(char *str);

//#include "uartlib0.c"

#endif
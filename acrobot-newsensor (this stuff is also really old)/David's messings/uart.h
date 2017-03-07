#ifndef UART_H_
#define UART_H_

#define RXD_PIN 0
#define TXD_PIN 1

#define LO(x) ((unsigned char) ((x) & 0x00ff))
#define HI(x) ((unsigned char) (((x) >> 8) & 0x00ff))
#define SEND16(x) uart_putchar(HI(x)); uart_putchar(LO(x));
#define SEND8(x) uart_putchar(x);

void uart_setup ();
int uart_getchar();
void uart_putchar (unsigned  c);

/* Prints out a null terminated string */
void uart_putstr(char *str);

void uart_putdata(uint8_t *data, uint8_t count);

#endif

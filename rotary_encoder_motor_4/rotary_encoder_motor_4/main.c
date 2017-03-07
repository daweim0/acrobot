/*
 * roatry_encoder_motor.cpp
 *
 * Created: 7/26/2016 2:21:52 PM
 * Author : David Michelman, daweim0@gmail.com
 */ 

#define F_CPU 16000000UL // 16 MHz clock speed
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define INA (1 << PORTD7)
#define PWM (1 << PORTD6)
#define INB (1 << PORTD5)
#define ENABLE (1 << PORTB0)
#define STEPS_PER_REV 2400  //The nuber of pulses per revoltuion from the rotary encoder


//None of these variables probably need to be volatile, but it makes debugging easier
// by not letting the compiler optimize them away.
volatile int32_t steps;
volatile char pins;
volatile int32_t time;


volatile float h1 = 1.0;
volatile float h2 = 1.0;
volatile float y1 = 0.0;
volatile float y2 = 0.0;
volatile float y3 = 0.0;
volatile float target_acceleration = 0.2;
volatile float ddx = 0;


int usart_putchar_printf(char var, FILE *stream);
void USART0SendByte(char u8Data);
uint8_t USART0ReceiveByte(void);
uint8_t USART0HasNext(void);

//Used by printf()
static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);

const char *byte_to_binary(int x)
{
	static char b[9];
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1)
	{
		strcat(b, ((x & z) == z) ? "1" : "0");
	}

	return b;
}

int main(void)
{
	
	EICRA |= (1 << ISC00) | (1 << ISC10);
	EIMSK |= (1 << INT0) | (1 << INT1);
	//DDRD = 0x00;	
	
	USART0Init();
	
	//Start timer without prescaller
	TCNT1 = 0;
	TCCR1B|= 1 << CS10 | 1 << CS11;	

	//Set up printf	
	stdout = &mystdout;
	
	//
	DDRB |= (1 << PORTB2);
	DDRB |= (1 << PORTB1);

	DDRB |= ENABLE;
	DDRD |= INA | INB | PWM;
	PORTD |= INA;
	PORTB |= ENABLE;
	printf("alive");
	
	sei();
	
	steps = 0;
	
	while(1) 
	{
		
		//if(USART0HasNext()) {
			//int8_t dataRaw = USART0ReceiveByte();
			//target_acceleration = (float) dataRaw / 10.0;
		//}
		
		//Read the current time and reset the timer
		uint16_t current_dt = TCNT1;
		TCNT1 = 0;

		//Calculate the angular acceleration of the elbow joint
		y1 = y2;
		y2 = y3;
		y3 = (float) steps / STEPS_PER_REV;
		h1 = h2;
		h2 = (float) current_dt / (F_CPU/ 64);

		ddx = 4 * (y3 - 2 * y2 + y1) / ((h1 + h2) * (h1 + h2));
		
		//Change the applied torque accordingly. This part needs serious tuning or
		// a complete rewrite
		if(target_acceleration < ddx) {
			PORTD &= ~INA;
			PORTD |= INB;
			PORTD |= PWM;
		}
		else {
			PORTD |= INA;
			PORTD &= ~INB;
			PORTD |= PWM;
		}
		
		//if(target_acceleration > 0) {
			//PORTD |= INA;
			//PORTD &= ~INB;
			//if(target_acceleration > ddx) {
				//PORTD &= ~PWM;
			//}
			//else {
				//PORTD |= PWM;
			//}
		//}
		//else {
			//PORTD &= ~INA;
			//PORTD |= INB;
			//if(target_acceleration < ddx) {
				//PORTD &= ~PWM;
			//}
			//else {
				//PORTD |= PWM;
			//}
		//}
		
		//PORTD &= ~INB;
		//PORTD |= INA;
		//PORTD |= PWM;
		//_delay_ms(5);
		//
		//PORTD &= ~INA;
		//PORTD |= INB;
		//PORTD |= PWM;
		//_delay_ms(5);

		 
		printf("% 2i ", steps);
		//printf(" %s \r\n", byte_to_binary(pins));
		printf(", %f", h2);
		printf(", %f \r\n", ddx);
		_delay_ms(50);
	}
}


void USART0Init(void) {
	// Set baud rate
	UBRR0H = 0;
	UBRR0L = 34;
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable transmission and reception
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	UCSR0A |= (1<<U2X0);   /* double speed */
}

void USART0SendByte(char u8Data) {
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	UDR0 = u8Data;
}

uint8_t USART0ReceiveByte() {
	// Wait for byte to be received
	while(!(UCSR0A&(1<<RXC0))){};
	// Return received data
	return UDR0;
}

uint8_t USART0HasNext() {
	return UCSR0A&(1<<RXC0);
}


int usart_putchar_printf(char var, FILE *stream) {
	USART0SendByte(var);
	return 0;
}


//Capture rotary encoder output
ISR (INT0_vect, ISR_NOBLOCK) {
	cli();
	_delay_us(2);
	pins = PIND;
	pins &= (1 << PORTD2) | (1 << PORTD3);
	//if(bit_get(pins, 1 << PORTD2)) {
		//PORTB |= 1 << PORTB2;
	//}
	//else {
		//PORTB &= ~(1 << PORTB2);
	//}
	
 	//if(pins & ~(1 << PORTD2)) {
		//steps += 1;
		//PORTB |= 1 << PORTB1;
		//_delay_us(500);
		//PORTB &= ~(1 << PORTB1);
	//}
	//else {
		//steps -= 1;
		//PORTB |= 1 << PORTB2;
		//_delay_us(500);
		//PORTB &= ~(1 << PORTB2);
	//}
	if(bit_get(pins, 1 << PORTD2)) {
		 // pin 3 is high
		 if(bit_get(pins, 1 << PORTD3)) {
			 // pin 4 is high
			 steps -= 1;
		 }
		 else {
			 // pin 4 is low
			 steps += 1;
		 }
	}
	
	else {
		// pin 3 is low
		if(bit_get(pins, 1 << PORTD3)) {
			// pin 4 is high
			steps += 1;
		}
		else {
			// pin 4 is low
			steps -= 1;
		}
	}

}

//Capture rotary encoder output
ISR (INT1_vect) {
	cli();
	_delay_us(2);
	pins = PIND;
	pins &= (1 << PORTD2) | (1 << PORTD3);
	if(bit_get(pins, 1 << PORTD2)) {
		// pin 3 is high
		if(bit_get(pins, 1 << PORTD3)) {
			// pin 4 is high
			steps += 1;
		}
		else {
			// pin 4 is low
			steps -= 1;
		}
	}
	
	else {
		// pin 3 is low
		if(bit_get(pins, 1 << PORTD3)) {
			// pin 4 is high
			steps -= 1;
		}
		else {
			// pin 4 is low
			steps += 1;
		}
	}
}

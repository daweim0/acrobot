#define	F_CPU 16000000

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <assert.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <stdio.h>

#include "mpu6050/mpu6050.h"
#include "PID_v1.h"

#define HIST_LEN 8
#define MAX_SUM_COUNT 1
#define CALIBRATION 127
#define LED_RED 1<<PB5
#define LED_BLUE 1<<PB6
#define LED_GREEN 1<<PB7

#define COMMAND_PREFIX 0x01  //a torque of one will probably never be applied (it wouldn't be able to overcome friction)
#define SET_LED_PREFIX 11
#define PING_PREFIX 12
#define SERVO_PREFIX 13

#define GYRO_OFFSET 31
#define GYRO_SCALAR 0.001070432
#define MINIMUM_DT 1/64.0  // 64 hz servo update frequency

#define SYSTEM_CLOCK_SCALAR 16000  // scales the cpu frequency (16 mhz) to milliseconds

#define setbit(port, bit) (port) |= (1 << (bit));
#define clearbit(port, bit) (port) &= ~(1 << (bit));
#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define true 1
#define false 0


/* Globals */
volatile uint16_t upper[HIST_LEN] = {0};
volatile uint16_t lower[HIST_LEN] = {0};
volatile uint16_t upper_period[HIST_LEN] = {0};
volatile uint16_t lower_period[HIST_LEN] = {0};
volatile uint16_t up_sum = 50;      // upper joint 4-sample sum
volatile uint16_t lo_sum = 50;      // lower joint 4-sample sum
volatile uint8_t up_sum_count = 4; // how many samples have been summed
volatile uint8_t lo_sum_count = 4; // how many samples have been summed
volatile uint8_t up_idx = 0;    // index for upper joint history
volatile uint8_t up_oldest = 0; // remembers latest upper index
//volatile uint8_t up_done = 0;   // true when have complete entry
volatile uint8_t lo_idx = 0;    // index for lower joint history
volatile uint8_t lo_oldest = 0; // remembers latest lower index
//volatile uint8_t lo_done = 0;   // true when have complete entry

uint8_t cmdLow;
uint8_t cmdHigh;
volatile uint16_t *up_target;
volatile uint16_t *lo_target;

volatile int64_t system_time = 0;  // the number of elapsed clock cycles as kept by TCNT1 (approximately)
volatile double last_time = 0;
volatile int16_t last_gyro_reading = 0;
volatile float target_acceleration = 0;
volatile uint16_t last_upper_reading = 0;
volatile bool servo_on = false;
volatile int8_t lastPower = -99;
volatile int8_t resulting_power = 0;


#if MPU6050_GETATTITUDE == 0
volatile int16_t ax = 0;
volatile int16_t ay = 0;
volatile int16_t az = 0;
volatile int16_t gx = 0;
volatile int16_t gy = 0;
volatile int16_t gz = 0;
volatile double axg = 0;
volatile double ayg = 0;
volatile double azg = 0;
volatile double gxds = 0;
volatile double gyds = 0;
volatile double gzds = 0;
#endif

#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
long *ptr = 0;
volatile double qw = 1.0f;
volatile double qx = 0.0f;
volatile double qy = 0.0f;
volatile double qz = 0.0f;
volatile double roll = 0.0f;
volatile double pitch = 0.0f;
volatile double yaw = 0.0f;
#endif



/* Function Prototypes */
//void mainOLD();
void setPower(uint8_t);
void init_all();

//command method prototypes
void ping();
void setLed(uint8_t);
void oldTransmissionHandler(int8_t);
float getAcceleration(void);
int usart_putchar_printf(char, FILE *stream);

// uartlib.h
void init_all(void);
void SEND16(uint16_t value);
void uart_setup();
void uart_putchar(uint8_t c);
void uart_putstr(char *str);
void uart_putdata(uint8_t *data, uint8_t count);
char uart_getchar(void);
uint8_t uart_hasNext(void);

volatile char printable;

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);


//////////////////////////////////////////////////
#define MOTOR_DRIVER_UART_BAUD 57600  //Baud rate used when communicating with the atmega328 motor driver

void setPower(uint8_t power)
{  
	/* 
		This function appears to run slightly fast, specifying a baud rate of 38400 bps gives 42780 bps.
		Maybe add an additional delay or try with higher optimization levels? (_delay_us() claims to only work with
		optimization, but the current level is -O1 so something fishy might be going on)
	*/
	cli();
	DDRA |= 1 << PA3;
	PORTA &= ~(1<<PA3);            // start bit
	for( uint8_t i = 10; i; i-- ){        // 10 bits
		_delay_us( 1e6 / MOTOR_DRIVER_UART_BAUD);            // bit duration
		if( power & 1 )
		PORTA |= 1<<PA3;        // data bit 0
		else
		PORTA &= ~(1<<PA3);           // data bit 1 or stop bit
		power >>= 1;
	}
	// set pin to pull up mode
	DDRA &= ~(1<<PA3);
	PORTA |= 1 << PA3;
	sei();
	
	//This code is used when the atmega128 directly controlls the motor driver, it is obsolete. 
	// A command of 0 is all stop.  The command is encoded as sign
	// magnitude where MSB is the sign bit and when high indicates
	// negative.  The lo seven bits encode the absolute value of
	// the power (torque) to apply to the motor.
	
	//if(power != CALIBRATION) {
		//setLed(0x01);
		//TCCR0 =
		//(1 << WGM00) | // phase correct PWM
		//(1 << COM01) | // non-inverting PWM
		//(1 << CS00);   // f/0 prescale -> 31.25kHz PWM (above audible (unless you happen to be a doge))
		////(1 << CS00);   // f/0 prescale -> 3.921kHz PWM (oh so audible (unless you don't have ears))
		////(1 << CS02);   // f/64 prescale -> .9804kHz PWM (audible but less annoying)
		//
		//PORTC = (1 << PC2);    // enables
		//if (!(power >> 7)) {PORTC |= (1 << PC1);} //set direction 1
		//else {PORTC |= (1 << PC0);} //set direction 2
		//// TODO figure out which one is left and right
//
		//power = power << 1;
		////if (9 < power < 30) {power = 30;}
//
		//OCR0 = power;                   // some speed ahead, scotty.
		////(remove sign bit and shove into tccr0 output compare)
	//}
	//else {                // All stop!
		//PORTC = 0;
		//TCCR0 = 0;
	//}
}


int8_t servoLoop(float target_acceleration) {
	//get current acceleration
	float current_acceleration = getAcceleration();
	if (current_acceleration == 999) {
		return 0;
	}
	//adjust torque appropriately
	//int8_t resulting_power = (target_acceleration - current_acceleration) / 25 * -1;
	int16_t current_angle = last_upper_reading / (F_CPU / 1000000);
	if(current_angle < 160) {
		resulting_power = 50;
	}
	else if(current_angle > 670) {
		resulting_power = -50;
	}
	else {
		//resulting_power = PID_Compute(current_acceleration, target_acceleration, system_time + TCNT1);
		
		if(target_acceleration > 0 && current_acceleration < target_acceleration) {
			resulting_power = 80;
			PORTF = 0x01;
		}
		else if(target_acceleration < 0 && current_acceleration > target_acceleration) {
			resulting_power = -80;
			PORTF = 0x01;
		}
		else {
			resulting_power = 0;
			PORTF = 0x01;
		}
		
	}
	//printf("target acceleration: % 2.3f,\tresulting power: % 4d,\tcurrent_acceleration: % 5.7f,\t", target_acceleration, resulting_power, current_acceleration);
	//printf("current angle: %i,\tITerm: %2.2f\t", current_angle, ITerm);
	//printf("\r\n");
	printf("%2.3f", current_acceleration);
	
	setbit(PORTF, PB1);
	for(int i = 0; i < (int8_t) abs(current_acceleration * 100); i++) {
		_NOP();
	}
	clearbit(PORTF, PB1);
	
	uint8_t processedPower = abs(resulting_power);
	if (resulting_power < 0) {
		processedPower |= 0b10000000;  // manually place the sign bit
	}
	setPower(processedPower);
	return 0;
}


float getAcceleration() {
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
	double current_time = (system_time + TCNT1) / ((float) F_CPU);
	double elapsed_time = current_time - last_time;
	
	if (elapsed_time < MINIMUM_DT) {
		return 999;
	}
	
	float current_velocity = (gx + GYRO_OFFSET) * GYRO_SCALAR;
	float last_velocity = (last_gyro_reading- GYRO_OFFSET) * GYRO_SCALAR;

	float acceleration = (current_velocity - last_velocity) / elapsed_time;

	last_time = current_time;
	last_gyro_reading = gx;

	//return acceleration;
	return current_velocity;
}


int main (void) {
	setLed(0b00000001); //blue
	
	// setup stream for printf (for debugging)
	uart_setup0();
	stdout = &mystdout;
	printf("alive");
	
	//MPU6050 configuration stuff
	mpu6050_init();
	_delay_ms(50);
	
	#if MPU6050_GETATTITUDE == 2
	mpu6050_dmpInitialize();
	mpu6050_dmpEnable();
	_delay_ms(10);
	#endif
	
	init_all();
	setPower(0);
	PID_Setup(15, 10, 0.5, -1, system_time);
	sei();                      // turn on interrupts
	
	setLed(0b00000010); //green
	
	printf(" entering main loop\r\n");
	
	DDRF = 0xFF;
	
	while (1) {
		if(uart_hasNext()){ //check to see if high byte of command is received
			cmdHigh = uart_getchar();
			
			
			//command parsing
			if(cmdHigh == COMMAND_PREFIX) {
				while(!uart_hasNext());	//loop until low byte is received
				cmdLow = uart_getchar();
				if(cmdLow == PING_PREFIX) {
					ping();
				}
				else if(cmdLow == SET_LED_PREFIX) {
					while(!uart_hasNext());
					setLed(uart_getchar());
				}
				else if (cmdLow == SERVO_PREFIX) {
					setLed(0x03);
					printf("got servo command\r\n");
					uint8_t data = uart_getchar();
					uint8_t sign = data >> 7;
					data &= 0x7f; // remove sign bit
					if (sign == 1) {
						data *= -1;
					}
					target_acceleration = ((float) ((int8_t) data)) / 10;
					printf("received data %f\r\n", target_acceleration);
					servo_on = true;
					oldTransmissionHandler(cmdHigh);
				}
				else {  // an invalid command was received
					setLed(0x04);
				}
			}
			// assume that it is a standard set torque and send angles back command
			else {
				
				if(cmdHigh == 0) {
					setLed(0x02);
				}
				else {
					setLed(0x01);
				}
				
				servo_on = false;
				printable = (cmdHigh << 1);
				printable = printable >> 1;
				if(cmdHigh >> 7 == 1) {printable *= -1;}
				setPower(printable);
				oldTransmissionHandler(cmdHigh);
				printf("set power to %i\r\n", printable);
			}
			
		}
		if (servo_on) {
			servoLoop(target_acceleration);
		}
		
		for(uint8_t i = 0; i < 0; i++) {
			_delay_loop_2(6553);
		}
		
	//setPower(0b1010000);
	_delay_us((1e6 / MOTOR_DRIVER_UART_BAUD) * 16);
	}
	return 0;
}


void oldTransmissionHandler(int8_t cmd){
	// Select which data to send back.  A command of CALIBRATION request
	// lower period data for calibration purposes.
	up_target = (cmd == CALIBRATION) ? upper_period : upper;
	lo_target = (cmd == CALIBRATION) ? lower_period : lower;

	// Now stream back data
	uint8_t i;
	uint16_t tmp;

	// make a copy so interrupts don't jinx the data...
	uint16_t up_cpy[HIST_LEN];  // high-time copy
	uint16_t lo_cpy[HIST_LEN];

	cli();
	up_oldest = up_idx;     // because idx always points one beyond newest
	lo_oldest = lo_idx;
	for (i = 0; i < HIST_LEN; i++) {
		up_cpy[i] = up_target[i];
		lo_cpy[i] = lo_target[i];
	}
	sei();

	// Transmit, oldest data first
	// send lower
	if (cmd == CALIBRATION) {
		printf("lower calibration data\r\n");
		for (i = 0; i < HIST_LEN; i++) {
			tmp = lo_cpy[(lo_oldest+i)%HIST_LEN];
			SEND16(tmp);
			printf("%i\r\n", tmp);
		}

		// send upper
		printf("upper calibration data\r\n");
		for (i = 0; i < HIST_LEN; i++) {
			tmp = up_cpy[(up_oldest+i)%HIST_LEN];
			SEND16(tmp);
			printf("%i\r\n", tmp);
		}
	}
	else {
		for (i = 0; i < HIST_LEN; i++) {
			tmp = lo_cpy[(lo_oldest+i)%HIST_LEN];
			SEND16(tmp);
		}

		// send upper
		for (i = 0; i < HIST_LEN; i++) {
			tmp = up_cpy[(up_oldest+i)%HIST_LEN];
			SEND16(tmp);
		}
	}
	
	if(cmd != CALIBRATION) {
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		//mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		
		//send raw gyro data
		SEND16(gx);
		SEND16(gy);
		SEND16(gz);
		SEND16(ax);
		SEND16(ay);
		SEND16(az);
	}
	
	//send processed(?) gyro data
	//uart_putdata(&gxds, 8);
	//uart_putdata(&gyds, 8);
	//uart_putdata(&gzds, 8);
	
}


// used by the computer to tell if the acrobot is plugged in and on
void ping(){
	uart_putchar(0x12);
}


//Sets the color of the status led, the three least significant bits in the argument controll Red, Green, and Blue respectively.
void setLed(uint8_t value){
	PORTB = PORTB & (~LED_BLUE || ~LED_GREEN || ~LED_RED);
	if(value)
	//turn red on
	if(value & 1<<2)
	PORTB |= LED_RED;
	
	//turn green on
	if(value & 1<<1)
	PORTB |= LED_GREEN;
	
	//turn blue on
	if(value & 1<<0)
	PORTB |= LED_BLUE;
}

int usart_putchar_printf(char var, FILE *stream) {
	uart_putchar0(var);
	return 0;
}

void init_all() {
	/***************************************************
	* Set up the interrupts for PWM input from the rotary encoders.
	* - Int4 and Int5 receive from the up joint.
	* - Int6 and Int7 receive from the lo joint.
	* - Even interrupts are rising edge triggered.
	* - Odd interrupts are falling edge triggered.
	*/
	EICRA = 0xFF;
	EICRB =
	(1 << ISC40) | (1 << ISC41) |
	(1 << ISC51) |
	(1 << ISC61) | (1 << ISC60) |
	(1 << ISC71);
	// Enable external interrupts 4-7.
	EIMSK = (1 << INT4) | (1 << INT5) | (1 << INT6) | (1 << INT7);

	/***************************************************
	* Set up the rotary encoder PWM reading 16-bit timers. whew. */
	/* Timer1 for reading up joint */
	TCNT1 = 0;
	TCCR1B |= (1 << CS10);      // no prescaling
	/* Timer3 for reading lo joint */
	TCNT3 = 0;
	TCCR3B |= (1 << CS30);      // no prescaling
	// no change to default
	
	//***************************************************
	// Set up the UART
	uart_setup();
	
	//**************************************************
	// Set output pins for motor control
	DDRB = (1 << PB4);          // PWM output compare pin
	DDRC =
	(1 << PC2) |            // ENable A/B
	(1 << PC1) |            // IN_A
	(1 << PC0);             // IN_B
	
	PORTC = (1 << PC2);    // enable the motor driver
	setPower(0);
}


/* UP JOINT ***********/
/* End of period rising edge */
ISR (INT4_vect/*, ISR_NOBLOCK*/) {
	if(TCNT1 < F_CPU / 1300) {  // will be false if an edge was missed
		upper_period[up_idx] = TCNT1; // get a period sample
	}
	system_time += TCNT1;
	TCNT1 = 0;                    // reset counter 1
	sei();
	// up_done = 1;
}

/* End of on.  Falling edge */
ISR (INT5_vect/*, ISR_NOBLOCK*/) {
	// Sum the first MAX_SUM_COUNT samples of a 16-sample period.  Up to 4
	// samples will fit into a 16-bit int because TCNT3 should be <= 16k.
	if (up_sum_count < MAX_SUM_COUNT)
	up_sum += TCNT1;
	
	// re-enable interrupts ASAP
	sei();
	
	up_sum_count += 1;

	// Store them after summation is complete and reset the sum.
	if (up_sum_count == MAX_SUM_COUNT) {
		upper[up_idx] = up_sum;
		last_upper_reading = up_sum / MAX_SUM_COUNT;
		up_sum = 0;
		up_idx = (up_idx + 1) % HIST_LEN;
		up_sum_count = 0;
	}
	// If up_sum_count starts larger than MAX_SUM_COUNT then reset it.
	if (up_sum_count > MAX_SUM_COUNT) {
		up_sum_count = 0;
		up_sum = 0;
	}
}

/* LO JOINT ***********/
/* End of period.  Rising edge */
ISR (INT6_vect/*, ISR_NOBLOCK*/) {
	if(TCNT3 < F_CPU / 1300 ) {  // will be false if an edge was missed
		lower_period[lo_idx] = TCNT3; // get latest period
	}
	TCNT3 = 0;                  /* reset counter 3 */
	sei();
	// lo_done = 1;
}

/* End of on.  Falling edge */
ISR (INT7_vect/*, ISR_NOBLOCK*/) {
	// Sum the first MAX_SUM_COUNT samples of a 16-sample period.  Up to 4
	// samples will fit into a 16-bit int because TCNT3 should be <= 16k.
	if (lo_sum_count < MAX_SUM_COUNT)
	lo_sum += TCNT3;
	
	// re-enable interrupts ASAP
	sei();

	lo_sum_count += 1;

	// Store them after summation is complete and reset the sum.
	if (lo_sum_count == MAX_SUM_COUNT) {
		lower[lo_idx] = lo_sum;
		lo_sum = 0;
		lo_idx = (lo_idx + 1) % HIST_LEN;
		lo_sum_count = 0;
	}
	// If up_sum_count starts larger than MAX_SUM_COUNT then reset it.
	if (lo_sum_count > MAX_SUM_COUNT) {
		lo_sum_count = 0;
		lo_sum = 0;
	}
}










// uartlib.c

uint8_t rxbuf[8];
volatile uint8_t rdptr = 0;
volatile uint8_t wrptr = 0;

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


// uartlib0.c

uint8_t rxbuf0[8];
volatile uint8_t rdptr0 = 0;
volatile uint8_t wrptr0 = 0;

void uart_setup0(void) {
	UBRR0H = 0;
	//UBRR0L = 0; // 2 mbps (with double speed)
	//UBRR0L = 1; // 1 mbps (with double speed taken into account)
	//UBRR0L = 3; // 500 kbps (with double speed taken into account)
	UBRR0L = 7; // 250 kbps (with double speed taken into account)
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
}

/**
* UART Receive ISR
*/
ISR(USART0_RX_vect) {
	rxbuf0[wrptr0] = UDR0;
	wrptr0 = (wrptr0 + 1) & 7; //& 7 forces wrptr to wrap
	if(wrptr0 == rdptr0)       //wrptr is data write head
	rdptr0 = (rdptr0 + 1) & 7; // rdprt is data read head
}

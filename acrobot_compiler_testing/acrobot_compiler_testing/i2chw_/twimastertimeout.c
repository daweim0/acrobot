

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <assert.h>
#include <util/delay.h>
#include <stdio.h>

#include "uartLib.h"
#include "uartlib0.h"
#include "mpu6050/mpu6050.h"


#define	F_CPU 16000000
#define HIST_LEN 8
#define MAX_SUM_COUNT 1
#define CALIBRATION 127
#define LED_RED 1<<PB5
#define LED_BLUE 1<<PB6
#define LED_GREEN 1<<PB7

#define COMMAND_PREFIX 0x01  //a torque of one will probably never be applied (it wouldn't be able to overcome friction)
#define SET_LED_PREFIX 11
#define PING_PREFIX 12

#define GYRO_OFFSET 28
#define GYRO_SCALAR 0.001070432

#define SYSTEM_CLOCK_SCALAR 16000  // scales the cpu frequency (16 mhz) to milliseconds

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
volatile int16_t last_time = 0;
volatile int16_t last_gyro_reading = 0;
volatile float target_acceleration = 0;
volatile uint16_t last_upper_reading = 0;

volatile uint16_t garbage = 0;


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
void setPower(int8_t);
void init_all();

//command method prototypes
void ping();
void setLed(uint8_t);
void oldTransmissionHandler(int8_t);
float getAcceleration(void);
int usart_putchar_printf(char, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);



//////////////////////////////////////////////////

int8_t servoLoop(float target_acceleration) {
	//get current acceleration
	float current_acceleration = getAcceleration();
	//adjust torque appropriately
	int8_t resulting_power = (target_acceleration - current_acceleration) * 100;
	int16_t current_angle = last_upper_reading;
	printf(current_angle);
	if(current_angle < 400) {
		setPower(-50);
	}
	else if(current_angle > 700) {
		setPower(50);
	}
	else {
		setPower((int8_t)target_acceleration);
	}
	return resulting_power;
}


float getAcceleration() {
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
	int16_t current_reading = gx;
	int64_t elapsed_time_fast = system_time + TCNT1 - last_time;
	

	float acceleration = ((current_reading - GYRO_OFFSET) * GYRO_SCALAR -
	(last_gyro_reading- GYRO_OFFSET) * GYRO_SCALAR) / (elapsed_time_fast);

	last_time = system_time + TCNT1;
	last_gyro_reading = current_reading;

	return acceleration;
}


int main (void) {
	setLed(0b00000100); //red
	
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
	
	printf("initializing everything");
	init_all();
	printf("setting power for the first time");
	setPower(0);
	printf("enabling interrupts");
	                      // turn on interrupts
	
	setLed(0b00000010); //green
	
	printf("entering main loop");
	sei();
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
				else {
					setLed(0x04);
				}
			}
			//assume that it is a standard set torque and send angles back command
			else {
				//if(cmdHigh != CALIBRATION ) {
					//target_acceleration = cmdHigh;
				//}
				oldTransmissionHandler(cmdHigh);
				setPower(cmdHigh);
			}
			
		}
		//printf("about to set acceleration");
		//servoLoop(target_acceleration);
	}
	return 0;
}

void setPower(int8_t power)
{
	// A command of 0 is all stop.  The command is encoded as sign
	// magnitude where MSB is the sign bit and when high indicates
	// negative.  The lo seven bits encode the absolute value of
	// the power (torque) to apply to the motor.
	
	if(power != CALIBRATION) {
		setLed(0x01);
		TCCR0 =
		(1 << WGM00) | // phase correct PWM
		(1 << COM01) | // non-inverting PWM
		(1 << CS00);   // f/0 prescale -> 31.25kHz PWM (above audible (unless you happen to be a doge))
		//(1 << CS00);   // f/0 prescale -> 3.921kHz PWM (very loud, quite annoying too)
		//(1 << CS02);   // f/64 prescale -> .9804kHz PWM (audible but less annoying)
		
		PORTC = (1 << PC2);    // enables
		if (!(power >> 7)) {PORTC |= (1 << PC1);} //set direction 1
		else {PORTC |= (1 << PC0);} //set direction 2
		// TODO figure out which one is left and right
		
		OCR0 = power << 1;                   // some speed ahead, scotty.
		//(remove sign bit and shove into tccr0 output compare)
		
		if(power == 0) {
			setLed(0x02);
		}
	}
	else {                // All stop!
		//PORTC = 0;
		//TCCR0 = 0;
	}
}


void oldTransmissionHandler(int8_t cmd){
	// Select which data to send back.  A command of CALIBRATION request
	// lower peridod data for calibration purposes.
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
	for (i = 0; i < HIST_LEN; i++) {
		tmp = lo_cpy[(lo_oldest+i)%HIST_LEN];
		SEND16(tmp);
	}

	// send upper
	for (i = 0; i < HIST_LEN; i++) {
		tmp = up_cpy[(up_oldest+i)%HIST_LEN];
		SEND16(tmp);
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


void ping(){
	uart_putchar(0x12);
}

//pre: Takes led values encoded as the 3 LSBs representing the R, G, and B values respectively
//post: sets the leds
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

}

/* UP JOINT ***********/
/* End of period rising edge */
ISR (INT4_vect/*, ISR_NOBLOCK*/) {
	upper_period[up_idx] = TCNT1; // get a period sample
	system_time = system_time + TCNT1 / SYSTEM_CLOCK_SCALAR;
	TCNT1 = 0;                    // reset counter 1
	printf("ISR4");
	sei();
	// up_done = 1;
}

/* End of on.  Falling edge */
ISR (INT5_vect/*, ISR_NOBLOCK*/) {
	// Sum the first MAX_SUM_COUNT samples of a 16-sample period.  Up to 4
	// samples will fit into a 16-bit int because TCNT3 should be <= 16k.
	if (up_sum_count < MAX_SUM_COUNT)
	up_sum += TCNT1;
	
	printf("ISR5");
	
	up_sum_count += 1;
	// re-enable interrupts ASAP
	sei();

	// Store them after summation is complete and reset the sum.
	if (up_sum_count == MAX_SUM_COUNT) {
		upper[up_idx] = up_sum;
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
	lower_period[lo_idx] = TCNT3; // get latest period
	garbage += 1;  // volatile variable access
	TCNT3 = 0;                  /* reset counter 3 */
	printf("ISR6");
	sei();
	// lo_done = 1;
}

/* End of on.  Falling edge */
ISR (INT7_vect/*, ISR_NOBLOCK*/) {
	// Sum the first MAX_SUM_COUNT samples of a 16-sample period.  Up to 4
	// samples will fit into a 16-bit int because TCNT3 should be <= 16k.
	if (lo_sum_count < MAX_SUM_COUNT)
	lo_sum += TCNT3;
	
	lo_sum_count += 1;

	printf("ISR7");

	// re-enable interrupts ASAP
	sei();

	// Store them after summation is complete and reset the sum.
	if (lo_sum_count == MAX_SUM_COUNT) {
		lower[lo_idx] = lo_sum;
		last_upper_reading = lo_sum;
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


//*********************************************
//old code archive
//*********************************************

//int8_t servoLoop(float target_acceleration) {
////get current acceleration
//float current_acceleration = getAcceleration();
////adjust torque appropriately
//int8_t resulting_power = (target_acceleration - current_acceleration) * 100;
//float current_angle = upper[up_idx] / MAX_SUM_COUNT;
//current_angle /= F_CPU * 1000;
//if(current_angle < 400) {
//setPower(-50);
//}
//else if(current_angle > 700) {
//setPower(50);
//}
//setPower((int8_t)target_acceleration);
//return resulting_power;
//}
//
//
//float getAcceleration() {
////mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
//int16_t current_reading = gx;
//int64_t elapsed_time_fast = system_time + TCNT1 - last_time;
//
//
//float acceleration = ((current_reading - GYRO_OFFSET) * GYRO_SCALAR -
//(last_gyro_reading- GYRO_OFFSET) * GYRO_SCALAR) / (elapsed_time_fast);
//
//last_time = system_time + TCNT1;
//last_gyro_reading = current_reading;
//
//return acceleration;
//}


//data send-back code
//// Select which data to send back.  A command of CALIBRATION request
//// lower peridod data for calibration purposes.
//up_target = (cmd == CALIBRATION) ? upper_period : upper;
//lo_target = (cmd == CALIBRATION) ? lower_period : lower;
//
//// Now stream back data
//uint8_t i;
//uint16_t tmp;
//
//// make a copy so interrupts don't jinx the data...
//uint16_t up_cpy[HIST_LEN];  // high-time copy
//uint16_t lo_cpy[HIST_LEN];
//
//// while (!up_done && !lo_done); // wait for end of periods
//
//cli();
//up_oldest = up_idx;     // because idx always points one beyond newest
//lo_oldest = lo_idx;
//for (i = 0; i < HIST_LEN; i++) {
//up_cpy[i] = up_target[i];
//lo_cpy[i] = lo_target[i];
//}
//sei();
//
//// Transmit, oldest data first
//// send lower
//for (i = 0; i < HIST_LEN; i++) {
//tmp = lo_cpy[(lo_oldest+i)%HIST_LEN];
//SEND16(tmp);
//}
//
//// send upper
//for (i = 0; i < HIST_LEN; i++) {
//tmp = up_cpy[(up_oldest+i)%HIST_LEN];
//SEND16(tmp);
//}
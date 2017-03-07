// A PID-servo controller

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#define LO(x) ((unsigned char) ((x) & 0x00ff))
#define HI(x) ((unsigned char) (((x) >> 8) & 0x00ff))
#define SEND16(x) uart_putchar(HI(x)); uart_putchar(LO(x));

#define H 0
#define T 1
#define Hp 2
#define Tp 3

/* Globals */
volatile uint16_t upper[4] = {0,0,0,0}; // upper joint pwm angles
volatile uint16_t lower[4] = {0,0,0,0}; // lower joint pwm angles
int16_t up = 0;
int16_t lo = 0;
int16_t v_up = 0;
int16_t v_lo = 0;
uint8_t power = 0;                      // motor speed control

/* Function Prototypes */
void init_all ();

//////////////////////////////////////////////////
int main(void) {
    

    return 0;
}



void init_all(void) {
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
    // Set up the UART!
    uart_setup();

    //**************************************************
    // Set output pins for motor control
    DDRB = (1 << PB4);          // PWM output compare pin
    DDRC =
        (1 << PC2) |            // ENable A/B
        (1 << PC1) |            // IN_A
        (1 << PC0);             // IN_B
}

// Returns angle normalized so 0 is (locally) vertical
int16_t get_angle(int16_t high, int16_t low) {
    int16_t angle = 1026 * high / total - 1;
    if (angle > 1022) angle = 1023;
    return angle - 512;
}

/* UP JOINT ***********/
/* rising edge, end of period*/
ISR (INT4_vect) {
    upper[Tp] = upper[T];       // copy previous value
    upper[T] = TCNT1;           /* read end of period time */
    TCNT1 = 0;                  /* reset counter 1 */
}

/* falling edge */
ISR (INT5_vect) {
    upper[Hp] = upper[H];       // copy previous value
    upper[H] = TCNT1;           /* read end of high time */
}

/* LO JOINT ***********/
/* rising edge */
ISR (INT6_vect) {
    lower[Tp] = lower[T];       // copy previous value
    lower[T] = TCNT3;           /* read end of period time */
    TCNT3 = 0;                  /* reset counter 3 */
}

/* falling edge */
ISR (INT7_vect) {
    lower[Hp] = lower[H];       // copy previous value
    lower[H] = TCNT3;           /* read end of high time */
}

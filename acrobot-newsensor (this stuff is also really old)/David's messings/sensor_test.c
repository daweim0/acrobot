#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#define LOWER 1
#define UPPER 2
#define HIST_LEN 64
#define UP_P_AV 16557
#define LO_P_AV 16537

/* Globals */
volatile uint16_t upper[2][HIST_LEN];
volatile uint16_t lower[2][HIST_LEN];
volatile uint16_t upper_period[2][HIST_LEN];
volatile uint16_t lower_period[2][HIST_LEN];
volatile uint8_t up_idx = 0;    // index for upper joint history
volatile uint8_t lo_idx = 0;
volatile uint8_t up_buf = 0;    // active buffer for recording
volatile uint8_t lo_buf = 0;
volatile uint8_t up_done = 0;   // set when a buffer has been filled
volatile uint8_t lo_done = 0;
volatile uint8_t streaming = 0; // flag

int16_t speed = 0;                  // motor speed control

/* Function Prototypes */
void init_all ();

int main (void) {
    init_all();
    sei();                      // turn on interrupts
    int16_t cmd = 0;

    while (1) {
        cmd = uart_getchar();

        if(cmd > 0) {
            switch(cmd) {
            case 'u':
                streaming = UPPER;
                break;
            case 'l':
                streaming = LOWER;
                break;
            case ']':
                streaming = 0;
            default:
                streaming = 0;
                break;
            }
        }

        // stream upper data
        if (streaming) {
            volatile uint16_t *joint;
            volatile uint16_t *joint_period;
            volatile uint8_t *done;

            if (streaming == LOWER) { // streaming lower?
                joint = lower[lo_buf];
                joint_period = lower_period[lo_buf];
                done = &lo_done;
            } else {
                joint = upper[up_buf];
                joint_period = upper_period[up_buf];
                done = &up_done;
            }

            while(!(*done));    // wait for the buffer to fill
            *done = 0;           // clear done status
            
            uint16_t i;              // generic index
            uint16_t tmp;
            
            ////////////////////
            // Send the data, oldest data first
            ////////////////////

            for (i = 0; i < HIST_LEN; i++) {
                tmp = joint[i];
                SEND16(tmp);
            }

            // Send lower periods
            for (i = 0; i < HIST_LEN; i++) {
                tmp = joint_period[i];
                SEND16(tmp);
            }
        }

    }
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

/* UP JOINT ***********/
/* End of period rising edge */
ISR (INT4_vect) {
    upper_period[up_buf][up_idx] = TCNT1;       // get latest period
    TCNT1 = 0;                  /* reset counter 1 */
    sei();
    //if (abs(upper_period[up_buf][up_idx] - UP_P_AV) < 15)
        up_idx++;

    // Switch buffers when current one is full
    if (up_idx == HIST_LEN) {
        up_idx = 0;
        up_buf ^= 1;
        up_done = 1;
    }
}

/* End of on.  Falling edge */
ISR (INT5_vect) {
    upper[up_buf][up_idx] = TCNT1;      /* read end of period time */
    sei();
}

/* LO JOINT ***********/
/* End of period.  Rising edge */
ISR (INT6_vect) {
    lower_period[lo_buf][lo_idx] = TCNT3;       // get latest period
    TCNT3 = 0;                  /* reset counter 3 */
    sei();
    //if (abs(lower_period[lo_buf][lo_idx] - LO_P_AV) < 10)
        lo_idx++;

    // Switch buffers when current one is full
    if (lo_idx == HIST_LEN) {
        lo_idx = 0;
        lo_buf ^= 1;
        lo_done = 1;
    }
}

/* End of on.  Falling edge */
ISR (INT7_vect) {
    lower[lo_buf][lo_idx] = TCNT3;      /* read end of period time */
    sei();
}

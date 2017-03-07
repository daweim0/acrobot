#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#define HIST_LEN 8
#define SAMPLES 8
#define MAX_SUM_COUNT 4
#define CALIBRATION 127
#define	F_CPU 16000000

/* Globals */
volatile uint16_t upper[HIST_LEN] = {0};
volatile uint16_t lower[HIST_LEN] = {0};
volatile uint16_t upper_period[HIST_LEN] = {0};
volatile uint16_t lower_period[HIST_LEN] = {0};
volatile uint16_t up_sum = 0;      // upper joint 4-sample sum
volatile uint16_t lo_sum = 0;      // lower joint 4-sample sum
volatile uint8_t up_sum_count = 0; // how many samples have been summed
volatile uint8_t lo_sum_count = 0; // how many samples have been summed
volatile uint8_t up_idx = 0;    // index for upper joint history
volatile uint8_t up_oldest = 0; // remembers latest upper index
//volatile uint8_t up_done = 0;   // true when have complete entry
volatile uint8_t lo_idx = 0;    // index for lower joint history
volatile uint8_t lo_oldest = 0; // remembers latest lower index
//volatile uint8_t lo_done = 0;   // true when have complete entry

/* Function Prototypes */
void init_all ();

//////////////////////////////////////////////////
int main (void) {
    init_all();
    sei();                      // turn on interrupts
    int16_t cmd = 0;
    volatile uint16_t *up_target;
    volatile uint16_t *lo_target;
    
    while (1) {
        //////////
        // wait for a power command
        while ((cmd = uart_getchar()) < 0);
        //////////

        if (cmd && cmd != CALIBRATION) {
            // A command of 0 is all stop.  The command is encoded as sign
            // magnitude where MSB is the sign bit and when high indicates
            // negative.  The lo seven bits encode the absolute value of
            // the power (torque) to apply to the motor.
            TCCR0 =
                (1 << WGM00) | // phase correct PWM
                (1 << COM01) | // non-inverting PWMp
                (1 << CS01);   // f/8 prescale -> 3.921kHz PWM
            PORTC = (1 << PC2);    // enable
            if (cmd >> 7)
                PORTC |= (1 << PC1);
            else
                PORTC |= (1 << PC0);
            OCR0 = cmd << 1;                   // some speed ahead, scotty.
        } else {                // All stop!
            PORTC = 0;
            PORTB = 0;
            TCCR0 = 0;
        }

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

        // while (!up_done && !lo_done); // wait for end of periods
        
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
ISR (INT4_vect/*, ISR_NOBLOCK*/) {
    upper_period[up_idx] = TCNT1; // get a period sample
    TCNT1 = 0;                    // reset counter 1
    sei();
    // up_done = 1;
}

/* End of on.  Falling edge */
ISR (INT5_vect/*, ISR_NOBLOCK*/) {
    // Sum the first MAX_SUM_COUNT samples of a 16-sample period.  Up to 4
    // samples will fit into a 16-bit int because TCNT1 should be <= 16k.
    if (up_sum_count < MAX_SUM_COUNT)
        up_sum += TCNT1;

    // re-enable interrupts ASAP
    sei();

    // Store them after summation is complete and reset the sum.
    if (up_sum_count == MAX_SUM_COUNT - 1) {
        upper[up_idx] = up_sum;
        up_sum = 0;
        up_idx = (up_idx + 1) % HIST_LEN;
    }
    
    // up_done = 0;
    up_sum_count = (up_sum_count + 1) % SAMPLES;
}

/* LO JOINT ***********/
/* End of period.  Rising edge */
ISR (INT6_vect/*, ISR_NOBLOCK*/) {
    lower_period[lo_idx] = TCNT3; // get latest period
   / TCNT3 = 0;                  /* reset counter 3 */
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

    // Store them after summation is complete and reset the sum.
    if (lo_sum_count == MAX_SUM_COUNT - 1) {
        lower[lo_idx] = lo_sum;
        lo_sum = 0;
        lo_idx = (lo_idx + 1) % HIST_LEN;
    }
    
    // lo_done = 0;
    lo_sum_count = (lo_sum_count + 1) % SAMPLES;
}

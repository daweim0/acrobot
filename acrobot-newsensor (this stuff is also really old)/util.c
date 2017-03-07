#include "util.h"

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

}

//**************************************************
// Set output pins for motor control
}void init_motor_control () {
    DDRB = (1 << PB4);          // PWM output compare pin
    DDRC =
        (1 << PC2) |            // ENable A/B
        (1 << PC1) |            // IN_A
        (1 << PC0);             // IN_B
}

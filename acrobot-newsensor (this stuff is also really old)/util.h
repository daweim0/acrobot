#ifndef UTIL_H_
#define UTIL_H_

void init_all(void);

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

    // Store them after summation is complete and reset the sum.
    if (lo_sum_count == MAX_SUM_COUNT - 1) {
        lower[lo_idx] = lo_sum;
        lo_sum = 0;
        lo_idx = (lo_idx + 1) % HIST_LEN;
    }
    
    // lo_done = 0;
    lo_sum_count = (lo_sum_count + 1) % SAMPLES;
}

#endif

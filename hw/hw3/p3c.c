#pragma vector=TIMER_A0_VECTOR
_interrupt void Timer_A2_ISR(void) {
    if (leap_cnt < 11915) {
        timer++;
        leap_cntr++;
    } else {
        leap_cnt = 0;
    }
}

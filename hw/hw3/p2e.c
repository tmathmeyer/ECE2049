void runtimerA2() {
    TA2CTL = TASSEL_1 + CNTL_0 + ID_1 + MC_1;
    TA2CCR0 = 32767;
    TA2CCTL0 = CCIE;
}
long unsigned int timer = 0;
#pragma vector=TIMERA_2_VECTOR
_interrupt void Timer_A2_ISR() {
    timer++;
}

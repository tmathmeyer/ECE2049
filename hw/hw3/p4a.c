//4MHz -> 2199 cycles
void runtimerA2() {
    TA2CTL = TASSEL_2 + MC_1 + ID_1;
    TA2CCR0 = 2199;
    TA2CCTL0 = CCIE;
}

long unsigned int timer = 0;
#pragma vector=TIMER_A2_VECTOR
_inturrupt void Timer_A2_ISR() {
    timer++;
}

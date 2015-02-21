void play_note(int freq) {
    P7SEL    |= BIT5;
    P7DIR    |= BIT5;
    TB0CTL    = (TBSSEL__ACLK|ID__1|MC__UP);
    TB0CTL   &= ~TBIE;
    TB0CCR0   = freq?32768/freq:0; // play note of this frequency
    TB0CCTL0 &= ~CCIE;
    TB0CCTL3  = OUTMOD_7;
    TB0CCTL3 &= ~CCIE;
    TB0CCR3   = TB0CCR0/2;
}


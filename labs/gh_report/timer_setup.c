void run_timer() {
    TA2CTL = TASSEL_1 + CNTL_0 + ID_1 + MC_1;
    TA2CCR0 = 16; 
    // this fires an interrupt once every 1/2048 of a second (0.000488 seconds)
    TA2CCTL0 = CCIE;
}


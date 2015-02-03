void sevenSegIO(char inVal, char DP) {
    char WRITE = DP ? BIT0 : 0x00;
    switch(inVal) {
        case '0': P2OUT = (WRITE | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
            break;
        case '1': P2OUT = (WRITE | BIT1 | BIT2);
            break;
        case '2': P2OUT = (WRITE | BIT1 | BIT2 | BIT4 | BIT5 | BIT7);
            break;
        case '3': P2OUT = (WRITE | BIT1 | BIT2 | BIT3 | BIT4 | BIT7);
            break;
        case '4': P2OUT = (WRITE | BIT2 | BIT3 | BIT6 | BIT7);
            break;
        case '5': P2OUT = (WRITE | BIT1 | BIT3 | BIT4 | BIT6 | BIT7);
            break;
        case '6': P2OUT = (WRITE | BIT1 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
            break;
        case '7': P2OUT = (WRITE | BIT1 | BIT2 | BIT3);
            break;
        case '8': P2OUT = (WRITE | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
            break;
        case '9': P2OUT = (WRITE | BIT1 | BIT2 | BIT3 | BIT4 | BIT6 | BIT7);
            break;
        case 'A': P2OUT = (WRITE | BIT1 | BIT2 | BIT3 | BIT5 | BIT6 | BIT7);
            break;
        case 'B': P2OUT = (WRITE | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
            break;
        case 'C': P2OUT = (WRITE | BIT1 | BIT4 | BIT5 | BIT6);
            break;
        case 'D': P2OUT = (WRITE | BIT2 | BIT3 | BIT4 | BIT5 | BIT7);
            break;
        case 'E': P2OUT = (WRITE | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
            break;
        case 'F': P2OUT = (WRITE | BIT1 | BIT5 | BIT6 | BIT7);
            break;
    }
}


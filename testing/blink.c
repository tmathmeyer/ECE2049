#include <msp430.h>
//#include "devices.h"


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog timer

    // Set digital IO control registers for  Port 1 Pin 0
    P1SEL = P1SEL & ~BIT0;          // Select P1.0 for digital IO
    P1DIR |= BIT0;          // Set P1.0 to output direction
    __disable_interrupt();          // Not using interrupts so disable them


    //configure_cap_buttons();

    //BBM buttons = get_active_buttons(0);

    //configure_cap_leds();

    while(1)   // forever loop
    {
        volatile unsigned int i;    // volatile to prevent optimization
        // by compiler

        P1OUT = P1OUT ^ BIT0;       // Toggle P1.0 using exclusive-OR
        //P1OUT = P1OUT & ~BIT0;    // Test whether logic logic 0 lights LED
        //P1OUT |= BIT0;        // or logic logic 1 lights LED

        i = 50000;    // SW Delay
        while (i != 0)    // could have used while (i)
            i--;
    }

    return 0;
}


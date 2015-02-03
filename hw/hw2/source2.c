#include "msp430.h"
#include <math.h>

void main(void)
{   // give the size of these variables
    float d, e; // declare d and e (both 32 bits)
    int lp=1; // declare lp and set its value to 1 (16 bits)
    long unsigned int sqSum; // declare sqSum (32 bits)

    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

    d = 0.75; // set d to 0.75
    e = 84.5; // set e to 84.5
    while (lp < 64) // do loop while lp is less than 64
    {
        sqSum += pow(lp,2); // add lp^2 to sqSum
        e=e/d; // divide e by d
        d*=2; // multiply d by 2
        lp=(lp<<1); // multiply lp by 2
    }
}

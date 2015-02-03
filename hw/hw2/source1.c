#include "msp430.h"

int main(void)
{   // give the size of these variable
    int j,k,indx=1;         // declare three 16 bit integers, and set indx to 1
    long unsigned int sqSum; // declare a lon unsigned int (32 bits)

    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

    j = 1;      // set j to 1
    k = 84;   // set k to 84
    while(indx < 64)    // do the following while index is less than 64
    {
        k=k/j;  // divide k by j and put the result back into k
        j*=2;   // multiply j by 2 (in place)
        sqSum += indx*indx; // add index squared to sqSum
        indx = indx<<1; // multiply index by 2
    }
}

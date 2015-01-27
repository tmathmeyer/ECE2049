
#include "devices.h"
#include "CTS_Layer.h"
#include "structure.h"

void configure_cap_buttons(void) {
    TI_CAPT_Init_Baseline(&keypad);
    TI_CAPT_Update_Baseline(&keypad, 5);
}

void configure_cap_leds(void) {
    P1SEL &= ~BITMASK;
    P1DIR |=  BITMASK;
    P1OUT &= ~BITMASK;
}

void configure_lcd_display(void) {

}

BBM get_active_buttons(BBM *write_to) {
    BBM result = 0;

    // TODO: rewrite this... it uses MALLOC
    struct Element *keypressed = (struct Element *)TI_CAPT_Buttons(&keypad);
    __no_operation();

    switch(keypressed -> inputBits) {
        case CBIMSEL_0:
            result |= 0x1 << 0;
            break;
        case CBIMSEL_1:
            result |= 0x1 << 1;
            break;
        case CBIMSEL_2:
            result |= 0x1 << 2;
            break;
        case CBIMSEL_3:
            result |= 0x1 << 3;
            break;
        case CBIMSEL_4:
            result |= 0x1 << 5;
            break;
    }
    return result;
}

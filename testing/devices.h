/*
 * devices.h
 *
 * Created on january 27th, 2015
 * Author: Ted Meyer (tmathmeyer@gmail.com)
 */

#ifndef _M_PERIPHERALS_H_
#define _M_PERIPHERALS_H_

#include <msp430.h>

// button bitmap
#define BBM char
#define BITMASK (BIT1|BIT2|BIT3|BIT4|BIT5)


// configure the capacitive buttons
void configure_cap_buttons(void);

//configure the LCD display
void configure_lcd_display(void);

// get the active buttons (or write them to a location)
// if the pointer is null, the value will be returned
// if the pointer is not null, # of buttons active will be returned
BBM get_active_buttons(BBM *write_to);

#endif

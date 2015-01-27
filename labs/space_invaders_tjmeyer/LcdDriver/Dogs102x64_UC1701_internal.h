/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// Dogs102x64_UC1701_internal.h - Internal definitions for the Dogs102x64 LCD
//                                display driver with a UC1701
//                                controller.
// Author:  N. DeMarinis (optimizations)
//
//*****************************************************************************

#ifndef __DOGS102X64_UC1701_INTERNAL_H__
#define __DOGS102X64_UC1701_INTERNAL_H__


//*****************************************************************************
//
// Parameters for tuning the performance characteristics of this driver
//
//*****************************************************************************
// Use only one manual draw call for the entire screen (using GrFlush()) instead
// of drawing to the display multiple times for each primitive.
#define SINGLE_DRAW

// Ensure that command and data writes to display are atomic by disabling ISRs
// before sending data.
//#define USE_ATOMIC_DRAW

// Use TI's driver library for all GPIO, SPI, and Timer operations
//#define USE_DRIVERLIB

// Use a variable backlight via a PWM
//#define USE_PWM_BACKLIGHT

//*****************************************************************************
//
// Defines for the pins that are used to communicate with the UC1701
//
//*****************************************************************************

/*
 * Pin assignment is as follows:
 *
 * RST		P5.7
 * CS		P4.7
 * CD		P5.6
 * SIMO		P4.1
 * SCLK		P4.3
 *
 */

// Pins from MSP430 connected to LCD
#define PIN_CD              BIT6
#define PIN_CS              BIT4
#define PIN_RST             BIT7
#define PIN_BLEN	        BIT6
#define PIN_SPI_SIMO        BIT1
#define PIN_SPI_CLK         BIT3

// Ports
#define CD_DIR          P5DIR
#define CD_OUT          P5OUT
#define CS_DIR          P7DIR
#define CS_OUT          P7OUT
#define RST_DIR			P5DIR
#define RST_OUT			P5OUT
#define BLEN_DIR		P7DIR
#define BLEN_SEL		P7SEL
#define BLEN_OUT		P7OUT

#define CD_RST_DIR      P5DIR
#define CD_RST_OUT      P5OUT
#define CS_BACKLT_DIR   P7DIR
#define CS_BACKLT_OUT   P7OUT
#define CS_BACKLT_SEL   P7SEL
#define SPI_SEL         P4SEL
#define SPI_DIR         P4DIR

#define SPI_REG_CTL0	UCB1CTL0
#define SPI_REG_CTL1	UCB1CTL1
#define SPI_REG_BRL		UCB1BR0
#define SPI_REG_BRH		UCB1BR1
#define SPI_REG_IFG		UCB1IFG
#define SPI_REG_STAT	UCB1STAT
#define SPI_REG_TXBUF	UCB1TXBUF
#define SPI_REG_RXBUF	UCB1RXBUF

#ifdef USE_PWM_BACKLIGHT

// PWM timer for backlight
#define BACKLIGHT_TIMER_CTL		TB0CTL
#define BACKLIGHT_TIMER_CCTL	TB0CCTL4
#define BACKLIGHT_TIMER_CCR		TB0CCR4
#define BACKLIGHT_TIMER_R		TB0R

// Capture compare count source for backlight timer
#define BACKLIGHT_TIMER_CCLOADR	  TB0CCR0
#define BACKLIGHT_TIMER_CCLOADCTL TB0CCTL0

// Period for backlight timer
#define BACKLIGHT_TIMER_CLKSRC (TBSSEL__ACLK)
#define BACKLIGHT_TIMER_PERIOD 50
#endif

/*
 * UCSI SPI Clock parameters
 * The actual clock frequency is given in number of
 * ticks of the specified clock source.
 *
 * For our configuration, we use the same frequency
 * as SMCLK, which is 1.25MHz.
 */
#define SPI_CLK_SRC		(UCSSEL__SMCLK)
#define SPI_CLK_TICKS	0

#endif

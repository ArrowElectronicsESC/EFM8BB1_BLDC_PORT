/*
 * timers.c
 *
 *  Created on: Jan 16, 2020
 *      Author: a92862
 */

#include "bldcdk.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void TMR_init_timer0(void);
void TMR_init_timer3(void);
void TMR_delay_timer3(U8 delay_ms);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

SEG_DATA U8 repeated_timer0;
SEG_DATA U8 timer0_state;
SEG_DATA UU16 timer0_next;

//-----------------------------------------------------------------------------
// TMR_init_timer0
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures the Timer0 as a 16-bit timer with auto-reload.
//-----------------------------------------------------------------------------
void TMR_init_timer0(void)
{
    // SYSCLK/1
    CKCON0 |= 0x04;           // T0M = 1
    TMOD &= ~0x04;           // CT0 = 0;
    TMOD |= 0x01;            // 16 bit mode
    TL0 = TH0 = 0;
    // start Timer0
    TCON_TR0 = 0;
}

//-----------------------------------------------------------------------------
// TMR_init_timer3
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
//  This function configures the Timer3 as a 16-bit timer with auto-reload,
//  SYSCLK/12, 1clk = 0.489us
//
//  during running state, this timer 3 is measuring phase current sampling
//  time to notify to PC.
//-----------------------------------------------------------------------------
void TMR_init_timer3(void)
{
    // Timer3 use SYSCLK/12
    CKCON0 &= ~0x40;             // T3ML = 0;
    CKCON0 &= ~0x80;             // T3MH = 0;
    TMR3CN0 &= ~0x01;            // T3XCLK = 0;    SYSCLK/12

    // Timer3 in 16-bit mode
    TMR3CN0 &= ~0x08;
    // Auto-reload value
    TMR3RLH = 0;
    TMR3RLL = 0;
}

//-----------------------------------------------------------------------------
// TMR_delay_timer3(delay_ms)
//
// Max 32ms delay
//
// Parameters:
//     delay_ms - number of 1 ms delay.
//
// Description:
//     Polling type delay routine.
//-----------------------------------------------------------------------------
void TMR_delay_timer3(U8 delay_ms)
{
    // Stop Timer 3
    TMR3CN0 &= ~0x04;

    TMR3RLH = ((-(SYSCLK/TIMER3_PRESCALER/1000)) >> 8) & 0xff;
    TMR3RLL = (-(SYSCLK/TIMER3_PRESCALER/1000)) & 0xff;

    TMR3 = TMR3RL;
    TMR3CN0 = 0x04;      // Start Timer 3
    for (; delay_ms > 0; --delay_ms)
    {
        while (!(TMR3CN0 & 0x80));   // Wait for overflow
        TMR3CN0 &= ~0x80;            // Clear the overflow indicator
    }
    // Stop Timer 3
    TMR3CN0 &= ~0x04;
}



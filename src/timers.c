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
void TMR_delay_timer3(U8 delay_ms);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

SEG_DATA U8 repeated_timer0;
SEG_DATA U8 timer0_state;
SEG_DATA UU16 timer0_next;

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



/*
 * timer.h
 *
 *  Created on: Jan 16, 2020
 *      Author: a92862
 */

#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_


//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "bldcdk.h"
//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define SYSCLK_KHZ              (SYSCLK/1000)
#define TIMER0_PRESCALER        48UL
#define TIMER2_PRESCALER        12UL
#define TIMER3_PRESCALER        12UL


#define TIMER0_SKIP_INDUCTIVE_KICK          0
#define TIMER0_ZERO_DETECTING               1
#define TIMER0_ZERO_CROSSING_DETECTED       2
#define TIMER0_COMMUTATION                  3
#define TIMER0_START_HYPERDRIVE             4

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern SEG_DATA U8 repeated_timer0;
extern SEG_DATA U8 timer0_state;
extern SEG_DATA UU16 timer0_next;
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void TMR_init_timer0(void);
void TMR_init_timer3(void);
void TMR_delay_timer3(U8 delay_ms);


#endif /* INC_TIMERS_H_ */

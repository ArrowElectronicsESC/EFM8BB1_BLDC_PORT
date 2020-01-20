//-----------------------------------------------------------------------------
// btn.c
//-----------------------------------------------------------------------------
// Copyright 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Created on: Jun 19, 2013
//     Author: khsoh
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "bldcdk.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
// Debouncing release time configuration - in units of 2.7ms
#define BTN_DEBOUNCE_TIME   8       // 8 == 20+ msecs

// Defines the return value of btn1_state
#define BTN_RELEASED    0
#define BTN_PRESSED     1
#define BTN_DEBOUNCING  2

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
static U8 SEG_XDATA btn1_state = BTN_RELEASED;
static U8 SEG_XDATA api_btn1_state = BTN_RELEASED;
U8 btn_released_time;

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
U8 is_btn1_pressed(void);
void btn_process(void);


//-----------------------------------------------------------------------------
// is_btn1_pressed
//-----------------------------------------------------------------------------
// Return Value:
//     0: btn1 is not pressed
//     1: btn1 is pressed.
//
// Parameters:
//     None
//
// Description:
//
//-----------------------------------------------------------------------------
U8 is_btn1_pressed(void)
{
    btn_process();

    if ( (btn1_state == BTN_PRESSED) && (api_btn1_state == BTN_RELEASED) )
    {
        api_btn1_state = btn1_state;
        return 1;
    }
    api_btn1_state = btn1_state;
    return 0;
}

//-----------------------------------------------------------------------------
// btn_process
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Function to be called regularly to detect button state and
// ensure button release is debounced
//-----------------------------------------------------------------------------
void btn_process(void)
{
    U8 curtime;

    if (IS_BTN1_PRESSED())
    {
        btn1_state = BTN_PRESSED;
    }
    else if (btn1_state != BTN_RELEASED)
    {
        curtime = (U8)SL_MTR_time();

        // Button not pressed - we would need to debounce
        if (btn1_state == BTN_PRESSED)
        {
            btn1_state = BTN_DEBOUNCING;
            btn_released_time = curtime;
        }
        else  // Must be debouncing state
        {
            if ( (curtime - btn_released_time) >= BTN_DEBOUNCE_TIME )
            {
                // Button is released when release has been debounced
                btn1_state = BTN_RELEASED;
            }
        }
    }
}

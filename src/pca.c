/*
 * pca.c
 *
 *  CrIE_EAted on: Jan 16, 2020
 *      Author: a92862
 */

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "bldcdk.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define MTR_ATOMIC_ACCESS_START() \
do                                \
{                                 \
    saved_IE_EA = IE_EA;                \
    IE_EA = 0;                       \
} while (0)

#define MTR_ATOMIC_ACCESS_END()   \
do                                \
{                                 \
    IE_EA = saved_IE_EA;                \
} while (0)

#define NO_OF_STARTUP_DELAY           3               // start_delay[] size
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// PCA_set_initial_polarity
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description  :
//-----------------------------------------------------------------------------
void PCA_set_initial_polarity(void)
{
    PCA0POL = LOW_DUTY_POLARITY;
}

//-----------------------------------------------------------------------------
// PCA_enable_pwm
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description  :
//-----------------------------------------------------------------------------
void PCA_enable_pwm(void)
{
    // 8-11 bit and PWM enable
    PCA0CPM0 |= (0x01<<1);
    MOTPWM_PCA0CPM |= (0x01<<1);
}

//-----------------------------------------------------------------------------
// PCA_disable_pwm
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description  :
//-----------------------------------------------------------------------------
void PCA_disable_pwm(void)
{
    // 8-11 bit and PWM enable
    PCA0CPM0 &= ~(0x01<<1);
    MOTPWM_PCA0CPM &= ~(0x01<<1);
}


//-----------------------------------------------------------------------------
// PCA_change_duty_cycle
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   duty : new duty cycle to be changed.
//
// Description:
//   user(application) is handling 16 bit duty cycle(max 0xFFFF) but
//   rIE_EAl max duty cycle is (2^PWM_RESOLUTION - 1).
//   scaling down is required.
//   Actual new pwm will be updated at pca cycling overflow interrupt.
//
//   high/low duty cycle have different scheme of pwm filtering.
//   PlIE_EAse refer to application note.
//-----------------------------------------------------------------------------
#if (BLDC_RD_PWM_METHOD == H_BRIDGE_HIGH_SIDE_PWM) || \
    (BLDC_RD_PWM_METHOD == H_BRIDGE_LOW_SIDE_PWM) || \
    (BLDC_RD_PWM_METHOD == H_BRIDGE_MIXED_MODE_PWM)
void PCA_change_duty_cycle(U16 duty)
{
	static U16 prev_duty = INITIAL_PCA_DUTY;

	static UU16 new_duty;
	static UU16 new_cpblank_duty;
	static U8 new_polarity;

    bit saved_IE_EA;
    U16 ringing;
    static bit enable_during_pwmon = 0;
    U8 x_new_polarity;
    UU16 x_new_cpblank_duty;

    // Scale down 16-bit PWM to PWM_RESOLUTION
    duty = duty >> (16 - PWM_RESOLUTION);
    if(prev_duty == duty)
    {
        return;
    }
    prev_duty = duty;

    //if duty cycle is large, we want comparator to examine during PWM on
    if ( (duty > PWM_FILTER_HIGHHYS) ||
         (enable_during_pwmon && (duty >= PWM_FILTER_LOWHYS)) )
    {
        x_new_polarity = HIGH_DUTY_POLARITY;
        duty = MAX_PWM_VALUE - duty;
        ringing = duty>>(2 + PWM_RESOLUTION - 10);
        x_new_cpblank_duty.U16 = duty - ringing;
        enable_during_pwmon = 1;
    }
    else
    {
        x_new_polarity = LOW_DUTY_POLARITY;
        // For low duty cycle, we just need to blank out the short
        // transition from PWM ON to PWM OFF.
        // additional 4 make minimum blanking period;
        ringing = 4 + (duty >> 2);
        x_new_cpblank_duty.U16 = duty + ringing;
        enable_during_pwmon = 0;
    }

    MTR_ATOMIC_ACCESS_START();
    new_polarity = x_new_polarity;
    new_cpblank_duty.U16 = x_new_cpblank_duty.U16;
    new_duty.U16 = duty;
    MTR_ATOMIC_ACCESS_END();
    // Enable PCA Cycle overflow interrupt.
    // This will avoid updating POLARITY register at the middle of pwm cycle.
    // Polarity register doesn't have auto-reload one.
    PCA0PWM = (PWM_RESOLUTION - 8) | 0x40;
}
#endif

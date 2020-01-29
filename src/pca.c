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
    saved_ea = IE_EA;                \
    IE_EA = 0;                       \
} while (0)

#define MTR_ATOMIC_ACCESS_END()   \
do                                \
{                                 \
    IE_EA = saved_ea;                \
} while (0)

#define NO_OF_STARTUP_DELAY           3               // start_delay[] size
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
U16 user_timer;
static U16 prev_duty = INITIAL_PCA_DUTY;
static UU16 new_duty;
static UU16 new_cpblank_duty;
static U8 new_polarity;

//-----------------------------------------------------------------------------
// initialize_pca
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// PWM-24kHz frequency, 10bit resolution, Edge aligned signal used by default.
//
// to check zero crossing time, virtual 32bit timer established.
// -user_timer(MSB) + PCA0(LSB)
// CEX0 is used for pwm filtering.
// CEX1 is for FET driving.
//-----------------------------------------------------------------------------
#if (BLDC_RD_PWM_METHOD == H_BRIDGE_HIGH_SIDE_PWM) || \
    (BLDC_RD_PWM_METHOD == H_BRIDGE_LOW_SIDE_PWM) || \
    (BLDC_RD_PWM_METHOD == H_BRIDGE_MIXED_MODE_PWM)
void PCA_initialize_pca(void)
{
    // PCA clock = SYS_CLK
    // Enable interrupt (CF flag)
    PCA0MD = (0x4<<1) | (0x01<<0);

    // CEX0, CEX1 edge aligned,
    // CEX0 for Comparator enable, CEX1 for driving motor
    PCA0CENT = 0x00;

    // 8-11 bit and PWM enable
    PCA0CPM0 |= (0x01<<1) | 0x08;   // Enable MATCH enable
    MOTPWM_PCA0CPM |= (0x01<<1) | 0x08;   // Enable MATCH enable

    // Setup number of bits for PWM
    PCA0PWM = PWM_RESOLUTION - 8;

    // PCAnPOL=0, value(PCA0CP) is proportional to low period. - Active low
    // PCAnPOL=1, value(PCA0CP) is proportional to high period. - Active high
    // With POL=0, when PCA0CP = 0, duty is 0%, always high
    //             when PCA0CP = all 1, duty is 100%, always low
    // pwm filter(force comparator not to trigger) signal : active low

    // CEX0 - comparator enable) - normal polarity
    // CEX1 - driving pwm signal, assume the initial duty is less
    //        than PWM_FILTER_LOWHYS.
    PCA0POL = LOW_DUTY_POLARITY;

    // Initial duty cycle, auto reload register should be initialized.
    // Enable comparator always...
    // ARSEL = 1, access auto-reload register
    PCA0PWM |= 0x80;
    PCA0CPL0 = (U8)0;
    PCA0CPH0 = (0>>8);

    MOTPWM_PCA0CPL = (U8)INITIAL_PCA_DUTY;
    MOTPWM_PCA0CPH = (INITIAL_PCA_DUTY>>8);
    // ARSEL = 0, access capture/compare registers directly
    PCA0PWM &= ~0x80;

    // enable overflow interrupt.
    EIE1 |= (0x01<<4);
    user_timer = 0;
}


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
#endif

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
    bit saved_ea;
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


//-----------------------------------------------------------------------------
// PCA0_ISR
//-----------------------------------------------------------------------------
//
// PCA0 ISR Content goes here. Remember to clear flag bits:
// PCA0CN0::PCA0CN0_CCF0 (PCA Module 0 Capture/Compare Flag)
// PCA0CN0::CCF1 (PCA Module 1 Capture/Compare Flag)
// PCA0CN0::CCF2 (PCA Module 2 Capture/Compare Flag)
// PCA0CN0::CF (PCA Counter/Timer Overflow Flag)
// PCA0PWM::COVF (Cycle Overflow Flag)
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (PCA0_ISR, PCA0_IRQn)
{
	static UU16 xpca_count;

	if ( (PCA0PWM & 0x60) == 0x60 )
	{
		// PCA Cycle counter overflow interrupt - this is enabled because
		// user needs to update duty cycle of both the motor PWM and
		// the blanking PWM signal

		PCA0PWM = (PWM_RESOLUTION - 8) | 0x80;
		// protect from high priority interrupt service
		IE_EA = 0;
		PCA0POL = new_polarity;
		PCA0CN0_CCF0 = 0;
		MOTPWM_CCF = 0;
		// This clIE_EArs the ECOM bit-causing PWM output to go to inactive state
		PCA0CPL0 = new_cpblank_duty.U8[LSB];
		// This sets the ECOM bit
		PCA0CPH0 = new_cpblank_duty.U8[MSB];
		// This clIE_EArs the ECOM bit-causing PWM output to go to inactive state
		MOTPWM_PCA0CPL = new_duty.U8[LSB];
		MOTPWM_PCA0CPH = new_duty.U8[MSB];
		PCA0PWM = (PWM_RESOLUTION - 8);
		// CEX1, match flag is set
		if (MOTPWM_CCF)
		{
			xpca_count.U8[LSB] = PCA0L;
			xpca_count.U8[MSB] = PCA0H;
			// Additional hIE_EAdroom due to delays in execution
			// in case we miss the match event (toggling CEXn), we ensure that
			// postponed(+32) event occur. Otherwise, it could make 1(one) 100%
			// or 0% pwm cycle once in a while. This is happening at very low
			// or very high duty cycle when the period between pca cycle overflow
			// and match event is very short.
			xpca_count.U16 += 32;
			MOTPWM_PCA0CPL = xpca_count.U8[LSB];
			MOTPWM_PCA0CPH = xpca_count.U8[MSB];
		}
		// CEX0, match flag is set
		if (PCA0CN0_CCF0)
		{
			xpca_count.U8[LSB] = PCA0L;
			xpca_count.U8[MSB] = PCA0H;
			// Additional hIE_EAdroom due to delays in execution
			// in case we miss the match event (toggling CEXn), we ensure that
			// postponed(+32) event occur. Otherwise, it could make 1(one) 100%
			// or 0% pwm cycle once in a while. This is happening at very low
			// or very high duty cycle when the period between pca cycle overflow
			// and match event is very short.
			xpca_count.U16 += 32;
			PCA0CPL0 = xpca_count.U8[LSB];
			PCA0CPH0 = xpca_count.U8[MSB];
		}
		IE_EA = 1;
		// for application level information.
		SLW_pwm_updated = 1;
	}

	if (PCA0CN0_CF)
	{
		// Disable global interrupts to ensure coherence
		// in upper 16-bits of timer by higher priority interrupt.
		IE_EA = 0;
		PCA0CN0_CF = 0;
		user_timer++;
		IE_EA = 1;

		if( 0 == ((U8)user_timer & pid_calc_interval) )
		{
			pid_flag = 1;
		}
	}
}

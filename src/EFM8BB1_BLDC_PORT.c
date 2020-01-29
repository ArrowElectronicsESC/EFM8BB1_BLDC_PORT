//-----------------------------------------------------------------------------
// PCA0_8Bit_PWM_Output.c
//-----------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include "bldcdk.h"

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
static U8 APP_tune_callback(U16 pwm, U16 disp);
static bit APP_reset(void);
void Port_Setup(void);

//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
    WDTCN = 0xDE;           // Disable watchdog timer
    WDTCN = 0xAD;
}
 
//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------

void main(void)
{
//	uint16_t delay_count;                       // Used to implement a delay

//	enter_DefaultMode_from_RESET();

//    for (delay_count = 15000; delay_count > 0; delay_count--);	// wait a moment

	CLKSEL = 0x0;
	Port_Setup();
	UART_init();
	ADC_initialize_adc();
	MCP_tune_callback = &APP_tune_callback;
	MCP_reset_callback = &APP_reset;
	MCP_init();
	SL_MTR_init();
	MTRAPP_init();

	IE_EA = 1;
	while (1)
	{
		ADC_task();
		btn_process();
		MTRAPP_task();
		MCP_task();
	}
}


//-----------------------------------------------------------------------------
// Port_Setup
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// P0.1~P0.2 => Input, Hall sensor or BEMF
// P0.3 => Input, VMY
// P0.4, P0.5 => UART
// P0.6 => Input, Current measurement, ADC
// P0.7 => Input, VDMC, ADC
// P1.0 ==> POT, ADC or PWM input
// P1.1 => Output for Led or SW101 (DIRECTION BUTTON)
// P1.2~P1.7 => Output, PWM for motor driving
// P2.0 => C2D and LED
// P2.1 => Input SWITCH(START/STOP) SW102
//-----------------------------------------------------------------------------
void Port_Setup(void)
{
    P0 = 0xFF;
    P1 = 0xFF;
    P2 = 0xFF;

    // P0.6(I_MEAS), P0.7(VMDC) analog
    // We leave P0.0-P0.3 digital but connected to comparator mux
    // so that we can use PWM to filter the pin by driving it low when required
    // P0.0~P0.2 : VMA, VMB, VMC, P0.3:VMY - refer to AN

    P0MDIN = PxMDIN_INIT_VAL & 0xFF;
    P1MDIN = (PxMDIN_INIT_VAL>>8) & 0xFF;

    // P0.4 UART TX : push pull
    P0MDOUT |= 0x10;

    // push pull for motor gate drive pins
    P1MDOUT |= MOTDRV_ALL_MASK;

    // Configure buttons.  If FEATURE_BTNx is not defined, the macro does not
	// generate any code.
//	CONFIG_BTN0();
//	CONFIG_BTN1();

    // skip all except UART and one of filtered terminal pins which will
    // detect ZC during startup.
    P0SKIP = DEFAULT_P0SKIP & (~FILTERED_A_MASK);
    P1SKIP = DEFAULT_P1SKIP;

    // Enable Xbar, UART and CEX0,1, DISABLE weak pull-up
    // VMA,VMB, VMC, VMY must not have any pull up resister.
    // refer to AN
    XBR0 = 1;
    XBR1 = 0x02;
    XBR2 |= 0xC0;

}

//------------------------------------------------------------------------------
// This call-back will be called by MCP_Task() when TUNE command is executed
//------------------------------------------------------------------------------
#ifdef FEATURE_PID_TUNE_FUNCTION
U16 SEG_XDATA C0;
U16 SEG_XDATA tune_disp;
U8 SEG_XDATA tune_cnt = 0;
U16 SEG_XDATA Y0, Amplitude;
U16 SEG_XDATA half_pu, Pu;
U16 SEG_XDATA last_half_pu;
U16 SEG_XDATA max_speed, last_max_speed;
U16 SEG_XDATA twait, twait0;
U16 SEG_XDATA speed, prev_speed;
U16 SEG_XDATA nKu, nKi, nKp;
U32 SEG_XDATA KA;
bit dec;
bit tune_in_progress;
extern struct pt SEG_XDATA mcp_tune;
#define TEST_CNT    20

#define KP_KONST    ((U32)(ROUND_DIV((U32)(65536UL*3.2*3.14),(4UL*256))))
#define KI_KONST    ((U32)ROUND_DIV( (U32)(65536UL*3.2*2.2*3.14), (4UL*256*((DEFAULT_PID_INTERVAL+1)))))

static U8 APP_tune_callback(U16 pwm, U16 disp)
{
    bit event = 0;
    C0 = pwm;
    tune_disp = disp;

    tune_in_progress = 1;
    PT_BEGIN(&mcp_tune);

    // wait until motor started...
    PT_WAIT_UNTIL(&mcp_tune, (MOTOR_RUNNING == SLR_motor_state));

    // motor started. wait stable running
    // set initial duty cycle(C0) for test
    SLR_pwm_duty = C0;
    PCA_change_duty_cycle(SLR_pwm_duty);

    twait0 = SL_MTR_time();
    twait = twait0;
    while((twait - twait0) < 5000)
    {
        twait = SL_MTR_time();
        PT_YIELD(&mcp_tune);
    }

    // start pid tuning.
    // save initial speed.
    Y0 = MCP_get_16bit_register(MCP_REG_PRESENT_MOTOR_SPEED);
    tune_cnt = 0;
    dec = 1;
    half_pu = 0L;
    max_speed = 0;

    // decrease duty cycle firstly.
    SLR_pwm_duty = C0 - tune_disp;
    PCA_change_duty_cycle(SLR_pwm_duty);
    while(tune_cnt < TEST_CNT)
    {
        PT_WAIT_UNTIL(&mcp_tune, ((U8)SL_MTR_time() & DEFAULT_PID_INTERVAL) == 0);
        speed = MCP_get_16bit_register(MCP_REG_PRESENT_MOTOR_SPEED);
        if(dec)
        {
            if( (speed+2) < Y0)
            {
                SLR_pwm_duty = C0 + tune_disp;
                event = 1;
            }
        }
        else
        {
            if( speed > (Y0+2) )
            {
                SLR_pwm_duty = C0 - tune_disp;
                event = 1;
            }
        }
        if(event)
        {
            PCA_change_duty_cycle(SLR_pwm_duty);
            twait = SL_MTR_time();
            dec = ~dec;
            tune_cnt++;
            half_pu = twait - last_half_pu;
            last_half_pu = twait;
            last_max_speed = max_speed;
            event = 0;
        }

        if( speed > max_speed)
        {
            max_speed = speed;
        }
        //PT_YIELD(&mcp_tune);
    }

    // end test
    Pu = half_pu*2;
    Amplitude = (last_max_speed-Y0);
#if 0
    MCP_debug_print("P%d\n", (U32)Pu);
    MCP_debug_print("A%d\n", (U32)Amplitude);
#endif

    // Ku = (4*d)/(a*3.14)
    // Tyreus-Luyben PI Variables.
    // Kp = Ku/3.2 = 4*d/(a*3.14*3.2)
    KA = KP_KONST*Amplitude;
    nKp = ROUND_DIV((((U32)tune_disp)<<16), KA);
    // Ki = Kp/(2.2*Pu) = 4*d/(a*3.14*3.2*2.2*Pu);
    KA = KA*Pu;
    nKi = ROUND_DIV((((U32)tune_disp)<<16), KA);

    MCP_set_16bit_register(MCP_REG_PROPORTIONAL_GAIN,nKp, 0);
    MCP_set_16bit_register(MCP_REG_INTEGRAL_GAIN,nKi, 0);

    //Done
    tune_in_progress = 0;
    PT_END(&mcp_tune);
    return 2;
}

#else
static U8 APP_tune_callback(U16 pwm, U16 disp)
{
    //Done
    disp = pwm;
    return 0;
}
#endif

//-----------------------------------------------------------------------------
// APP_reset
//-----------------------------------------------------------------------------
//
// Return Value : 0
// Parameters   : None
// Description: This function called when PC send 'RST\n' cmd
//
//-----------------------------------------------------------------------------
static bit APP_reset(void)
{
    SL_MTR_stop_motor();
    return 0;
}


//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

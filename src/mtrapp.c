/*
 * mtrapp.c
 *
 *  Created on: Jan 20, 2020
 *      Author: a92862
 */

#include "bldcdk.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define MTRAPP_ATOMIC_ACCESS_START()     \
do                                       \
{                                        \
    saved_ea = EA;                       \
    EA = 0;                              \
} while (0)

#define MTRAPP_ATOMIC_ACCESS_END()       \
do                                       \
{                                        \
    EA = saved_ea;                       \
} while (0)


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// Additional motor current to accumulate to perform further averaging
#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
static SEG_DATA U16     prev_target_rpm;
#endif
static SEG_XDATA U16     read_speed_interval;

#if BLDC_RD_RPM_PWM_SRC == PWM_SPEED_SOURCE
SEG_XDATA UU32          prev_edge;
SEG_XDATA UU32          new_edge;
bit                     last_edge_event;
#endif
static bit speed_ctrl_by_pc;
static bit start_by_pc;
static SEG_XDATA U16    pot_reading;

//-----------------------------------------------------------------------------
// MTRAPP_init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// initialization for application level.
//-----------------------------------------------------------------------------
void MTRAPP_init()
{
    read_speed_interval = DEFAULT_SPEED_ADC_INTERVAL;
    SLW_oc_debounce = MCP_get_8bit_register(MCP_REG_OVER_CURRENT_PERSISTANCE);
    SLW_current_limit = MCP_get_8bit_register(MCP_REG_OVER_CURRENT_THRESHOLD);
    SLW_motor_max_rpm = MCP_get_16bit_register(MCP_REG_MAXIMUM_OPERATIONAL_SPEED);
    speed_ctrl_by_pc = 0;
    start_by_pc = 0;
#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
    SLW_target_rpm = 0;
#endif
    SLW_target_pwm_duty = 0;
    pot_reading = 0;
#if BLDC_RD_RPM_PWM_SRC == PWM_SPEED_SOURCE
    // enable port match interrupt
    EIE1 |= (0x01<<1);
#endif
}

//-----------------------------------------------------------------------------
// MTRAPP_task
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Application level task.
// NOTE : application should call SL_MTR_motor() regularly to keep motor
// running.
//-----------------------------------------------------------------------------
void MTRAPP_task()
{
#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
    U16 SEG_XDATA Pg, Ig;
#endif
    static U8 last_report;
    U8 tmp;
    bit run_dir;

    // keep motor running and manage state of motor
    // NOTE : application should call this regularly.
    SL_MTR_motor();

    tmp = is_btn1_pressed();
    if( MOTOR_STOPPED == SLR_motor_state )
    {
        if (tmp || start_by_pc)
        {
            if( !SLR_motor_stalled)
            {
                if(ADC0CN0 & 0x90)
                {
                    // SL_MTR_start_motor need CPT0MX_IMEASURE pin which is
                    // same as IMEAS_ADCMX.
                    // wait if adc in progress by ADC_task().
                	while(!ADC0CN0_ADINT);
                    // disconnect all pins
                    ADC0MX = 0x1F;
                }

                if (tmp)
                {
                    // Button 1 pressed - we reset speed_ctrl_by_pc
                    speed_ctrl_by_pc = 0;
                }

                SL_MTR_start_motor();
                // Configure LED0 here - in case we share LED with buttons
                // In ref design, BTN0 and LED0 are shared.
                // If FEATURE_LED0 is not defined, this macro does not
                // generate any code.
                CONFIG_LED0();
                // update direction register.
                MCP_set_8bit_register(MCP_REG_PRESENT_MOTOR_DIRECTION, \
                        SLW_user_direction, 0);
            }
            start_by_pc = 0;
        }

        // --------------------------------------------------------------------
        // read motor direction changes
        MTRAPP_read_direction();

        // --------------------------------------------------------------------
        // followings can be move to mtrapp_init(), if application need only
        // default(or fixed) value.
        SLW_oc_debounce = MCP_get_8bit_register(MCP_REG_OVER_CURRENT_PERSISTANCE);
        SLW_current_limit = MCP_get_8bit_register(MCP_REG_OVER_CURRENT_THRESHOLD);
        SLW_motor_max_rpm = MCP_get_16bit_register(MCP_REG_MAXIMUM_OPERATIONAL_SPEED);
        SL_MTR_change_num_poles( MCP_get_8bit_register \
                (MCP_REG_MOTOR_POLE_PAIRS_COUNT) * 2 );
#if BLDC_RD_RPM_OR_PWM == PWM_PARAMETER
        SLW_acceleration_step_size = \
                MCP_get_16bit_register(MCP_REG_ACCELERATION_STEP_SIZE);
        SLW_deceleration_step_size = \
                MCP_get_16bit_register(MCP_REG_DECELERATION_STEP_SIZE);
#endif

#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
        Pg = MCP_get_16bit_register(MCP_REG_PROPORTIONAL_GAIN);
        Ig = MCP_get_16bit_register(MCP_REG_INTEGRAL_GAIN);
        SL_MTR_change_pid_gain(Pg, Ig);
#endif
    }
    else if( MOTOR_RUNNING == SLR_motor_state )
    {
#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
        if (tmp || !SLW_target_rpm )
#elif BLDC_RD_RPM_OR_PWM == PWM_PARAMETER
        if (tmp || !SLW_target_pwm_duty )
#endif

        {
            SL_MTR_stop_motor();
#if defined(FEATURE_HYPERDRIVE)
            // The following does nothing if FEATURE_LED0 is not enabled
            LED0_OFF();
#endif
            // Re-configure BTN0 because this is shared with LED0 in
            // reference design kit.
            //CONFIG_BTN0();
        }
    }
    // Enable stream data even motor stopped.
    tmp = (U8)SL_MTR_time();
    if((last_report != tmp) && (tmp & RPM_REPORT_PERIOD == RPM_REPORT_PERIOD))
    {
        last_report = tmp;
        MCP_set_16bit_register(MCP_REG_PRESENT_MOTOR_SPEED,
                SLR_motor_current_rpm, 0);
        MCP_set_16bit_register(MCP_REG_ACTIVE_PWM_DUTY_CYCLE, SLR_pwm_duty, 0);
        run_dir = MCP_get_8bit_register(MCP_REG_PRESENT_MOTOR_DIRECTION);
        MCP_set_8bit_register(MCP_REG_PRESENT_MOTOR_DIRECTION, \
                run_dir, 0);
    }

    // -----------------------------------------------------------------------
    // read new target speed
    MTRAPP_read_target_speed();

    // -----------------------------------------------------------------------
    // update average current to register 0x32.
    MCP_set_16bit_register(MCP_REG_MOTOR_COIL_CURRENT,SLR_motor_current,0);

    // -----------------------------------------------------------------------
    // read Vmotor and update register. 0x34
    // update reg 0x34
#ifdef FEATURE_MEAS_VMDC
    MCP_set_16bit_register(MCP_REG_MOTOR_OPERATING_VOLTAGE,SLR_motor_voltage, 0);
#endif
}

//-----------------------------------------------------------------------------
// MTRAPP_read_direction
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None but it will change global variable SLW_user_direction.
//
//  read NEXT motor running direction according to build parameter.
//  In case of DIRECTION_BY_BUTTON, when button released
//  SLW_user_direction is updated.
//-----------------------------------------------------------------------------
void MTRAPP_read_direction(void)
{
    static bit wait_release=1;

    if(wait_release == 1)
    {
        if( IS_BTN0_PRESSED() )
        {
            wait_release = 0;
        }
    }
    else if( !IS_BTN0_PRESSED() && (wait_release == 0))
    {
        SLW_user_direction = ~SLW_user_direction;
        MCP_set_8bit_register(MCP_REG_TARGET_MOTOR_DIRECTION, \
                (U8)SLW_user_direction, 0);
        wait_release = 1;
    }
    // read register. 0x02 (0:CW, 1:CCW)
    SLW_user_direction = MCP_get_8bit_register(MCP_REG_TARGET_MOTOR_DIRECTION);
}


//-----------------------------------------------------------------------------
// MTRAPP_read_target_speed
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None but it will change global variable SLW_target_rpm.
//
//  set SLW_target_rpm or SLW_target_pwm_duty according to build-time parameter.
//  Those target values are saved to register accordingly.
//  To reach 100% of target duty cycle or SLW_motor_max_rpm in any case,
//  just added small constant to the adc result.
//-----------------------------------------------------------------------------
void MTRAPP_read_target_speed(void)
{
#if BLDC_RD_RPM_PWM_SRC == POT_SPEED_SOURCE
#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
    static U16 last_read_speed_time;
    U16 temp;

    if( ((U16)SL_MTR_time() - last_read_speed_time) < read_speed_interval )
    {
        return;
    }
    last_read_speed_time = (U16)SL_MTR_time();

    if (speed_ctrl_by_pc || ( MOTOR_STOPPED == SLR_motor_state ))
    {
        // read reg 0x3C (target rpm)
        temp = MCP_get_16bit_register(MCP_REG_TARGET_MOTOR_SPEED);
        if( SLW_target_rpm != temp )
        {
            if (temp == 1)
            {
                // Special case of target speed of 1.
                start_by_pc = 1;

                // Restore target speed
                MCP_set_16bit_register(MCP_REG_TARGET_MOTOR_SPEED, SLW_target_rpm, 0);
            }
            else
            {
                SLW_target_rpm = temp;
            }
            speed_ctrl_by_pc = 1;
        }
    }

    // read latest adc result....
    temp = adc_result[ADC_MUX_POT];
    // CW : Increase, CCW : Decrease speed
    temp = MAX_ADC_VALUE - temp;
    // Just added some amount to reach SLW_motor_max_rpm(or 100% duty) in any case.
    temp += (MAX_ADC_VALUE>>5);      // about 3%

    pot_reading = ((U32)(SLW_motor_max_rpm * (U32)temp))>>ADC_RESOLUTION;
    pot_reading = (prev_target_rpm + pot_reading)>>1;
    prev_target_rpm = pot_reading;
    if( pot_reading < APP_MOTOR_MIN_RPM )
    {
        pot_reading = APP_MOTOR_MIN_RPM;
    }

    if ( !speed_ctrl_by_pc )
    {
        // Use POT when speed is not controlled by PC
        SLW_target_rpm = pot_reading;
        // set register
        MCP_set_16bit_register(MCP_REG_TARGET_MOTOR_SPEED, SLW_target_rpm, 0);
    }

#elif BLDC_RD_RPM_OR_PWM == PWM_PARAMETER
    static U16 last_read_speed_time;
    U16 temp;

    if( ((U16)SL_MTR_time() - last_read_speed_time) < read_speed_interval )
    {
        return;
    }
    last_read_speed_time = (U16)SL_MTR_time();

    if (speed_ctrl_by_pc || ( MOTOR_STOPPED == SLR_motor_state ))
    {
        // read reg 0x3C (target rpm)
        temp = MCP_get_16bit_register(MCP_REG_TARGET_PWM_DUTY_CYCLE);
        if( SLW_target_pwm_duty != temp )
        {
            if (temp == 1)
            {
                // Special case of target speed of 1.
                start_by_pc = 1;

                // Restore target PWM duty cycle
                MCP_set_16bit_register(MCP_REG_TARGET_PWM_DUTY_CYCLE, SLW_target_pwm_duty, 0);
            }
            else
            {
                SLW_target_pwm_duty = temp;
            }
            speed_ctrl_by_pc = 1;
        }
    }

    // read latest adc result....
    temp = adc_result[ADC_MUX_POT];
    // CW : Increase, CCW : Decrease speed
    temp = MAX_ADC_VALUE - temp;
    // Just added some amount to reach SLW_motor_max_rpm(or 100% duty) in any case.
    temp += (MAX_ADC_VALUE>>5);      // about 3%

    if( temp > MAX_ADC_VALUE)
    {
        pot_reading = MAX_USER_PWM_VALUE;
    }
    else
    {
        // target pwm = 16 bit value.
        pot_reading = (temp<<(16-ADC_RESOLUTION));
    }

    if ( !speed_ctrl_by_pc )
    {
        // Use POT when speed is not controlled by PC
        SLW_target_pwm_duty = pot_reading;
        // set register (16bit pwm duty cycle)
        MCP_set_16bit_register(MCP_REG_TARGET_PWM_DUTY_CYCLE, \
                               SLW_target_pwm_duty, 0);
    }

#endif
#elif BLDC_RD_RPM_PWM_SRC == PWM_SPEED_SOURCE
    MTRAPP_read_pwm_input();
#endif
}


#if BLDC_RD_RPM_PWM_SRC == PWM_SPEED_SOURCE
//-----------------------------------------------------------------------------
// MTRAPP_read_pwm_input
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
//  Assuming the input frequency is 50~100Hz (20ms ~ 10ms)
//  Measuring % of high period.
//  Ex. target_rpm = MOTOR_MAX_RPM*high_period/(high_period+low_period)
//-----------------------------------------------------------------------------
void MTRAPP_read_pwm_input(void)
{
    UU32 SEG_XDATA tmp1, tmp2;
    UU16 SEG_XDATA period;
    U16 SEG_XDATA period_low, period_high;
    U16 temp;
    bit edge_event;

    EA = 0;
    edge_event = last_edge_event;
    tmp1.U32 = prev_edge.U32;
    tmp2.U32 = new_edge.U32;
    EA = 1;

    // to avoid overflow at high speed (200K rpm)
    period.U16 = (U16)((tmp2.U32 - tmp1.U32)>>8);
    // rising edge (end or start of period)
    if( edge_event)
    {
        period_low = period.U16;
    }
    // falling edge (end of duty)
    else
    {
        period_high = period.U16;
    }

#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
    if (speed_ctrl_by_pc || ( MOTOR_STOPPED == SLR_motor_state ))
    {
        temp = MCP_get_16bit_register(MCP_REG_TARGET_MOTOR_SPEED);
        if( SLW_target_rpm != temp )
        {
            if (temp == 1)
            {
                // Special case of target speed of 1.
                start_by_pc = 1;

                // Restore target speed
                MCP_set_16bit_register(MCP_REG_TARGET_MOTOR_SPEED, SLW_target_rpm, 0);
            }
            else
            {
                SLW_target_rpm = temp;
            }
            speed_ctrl_by_pc = 1;
        }
    }

    pot_reading = (U16)( ((U32)SLW_motor_max_rpm * period_high ) / \
            (U32)( period_high+period_low) );
    // Just added some amount.
    pot_reading += (MOTOR_MAX_RPM>>5);
    if(pot_reading < APP_MOTOR_MIN_RPM)
        pot_reading = APP_MOTOR_MIN_RPM;

    if ( !speed_ctrl_by_pc )
    {
        // Use POT when speed is not controlled by PC
        SLW_target_rpm = pot_reading;
        // set register
        MCP_set_16bit_register(MCP_REG_TARGET_MOTOR_SPEED, SLW_target_rpm, 0);
    }

#elif BLDC_RD_RPM_OR_PWM == PWM_PARAMETER
    if (speed_ctrl_by_pc || ( MOTOR_STOPPED == SLR_motor_state ))
    {
        // read reg 0x3C (target rpm)
        temp = MCP_get_16bit_register(MCP_REG_TARGET_PWM_DUTY_CYCLE);
        if( SLW_target_pwm_duty != temp )
        {
            if (temp == 1)
            {
                // Special case of target speed of 1.
                start_by_pc = 1;

                // Restore target PWM duty cycle
                MCP_set_16bit_register(MCP_REG_TARGET_PWM_DUTY_CYCLE, SLW_target_pwm_duty, 0);
            }
            else
            {
                SLW_target_pwm_duty = temp;
            }
            speed_ctrl_by_pc = 1;
        }
    }

    pot_reading = (U16)( ( (U32)MAX_USER_PWM_VALUE * period_high ) / \
            (U32)( period_high + period_low ) );
    // Just added some amount.
    pot_reading += (MAX_USER_PWM_VALUE>>5);
    if(pot_reading < (MAX_USER_PWM_VALUE>>5))
        pot_reading = MAX_USER_PWM_VALUE;
    else if(pot_reading < SLR_minimum_duty.U16)
        pot_reading = SLR_minimum_duty.U16;


    if ( !speed_ctrl_by_pc )
    {
        // Use POT when speed is not controlled by PC
        SLW_target_pwm_duty = pot_reading;
        // set register (16bit pwm duty cycle)
        MCP_set_16bit_register(MCP_REG_TARGET_PWM_DUTY_CYCLE, \
                               SLW_target_pwm_duty, 0);
    }
#endif
}
INTERRUPT(PORT_MATCH_ISR, INTERRUPT_PORT_MATCH)
{
    static U8 ovf;

    prev_edge.U32 = new_edge.U32;
    SL_MTR_GET_32BIT_TIME(new_edge);

    last_edge_event = (PWMIN_MAT != (1<<PWMIN_BIT));
    // set next port miss match event
    PWMIN_MAT ^= (1<<PWMIN_BIT);
}
#endif


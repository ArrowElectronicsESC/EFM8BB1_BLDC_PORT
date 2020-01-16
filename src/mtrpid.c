/*
 * mtrpid.c
 *
 *  Created on: Jan 16, 2020
 *      Author: a92862
 */


#include "bldcdk.h"

#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
static U16 newpwm;
static U16 lastspeed;
static U16 curspeed;
static U16 SEG_XDATA Kp;
static U16 SEG_XDATA Ki;
static UU32 SEG_XDATA tmpval;
#endif
bit pid_done;

static struct pt SEG_XDATA pidth;

static U8 MTR_pid_compute_thread(void);

#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER


//------------------------------------------------------------------------------
// void SL_MTR_change_pid_gain(U16 proportion, U16 integral)
//------------------------------------------------------------------------------
// Return Value:
//     None
//
// Parameters:
//     proportion: pid proportional gain (Ka)
//     integral  : pid integral gain (Ki)
//
// Description:
//     This function allowed for application level to update gains.
//     This will be called by application level only.
//------------------------------------------------------------------------------
void SL_MTR_change_pid_gain(U16 proportion, U16 integral)
{
    Kp = proportion;
    Ki = integral;
}

//------------------------------------------------------------------------------
// void MTR_pid_init(void)
//------------------------------------------------------------------------------
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     This function initializes the PI control variables.This should be called
//     at the start, or when the PI gain control variables have been changed.
//     It should also be run if the PI control task has not been executed for
//     a long time.
//------------------------------------------------------------------------------
void MTR_pid_init(void)
{
    lastspeed = SLR_motor_current_rpm;
    pid_done = 0;
//    Kp = MCP_get_16bit_register(MCP_REG_PROPORTIONAL_GAIN);
//    Ki = MCP_get_16bit_register(MCP_REG_INTEGRAL_GAIN);

    PT_INIT(&pidth);
}



//------------------------------------------------------------------------------
// U16 MTR_pid_compute_newpwm(void)
//------------------------------------------------------------------------------
// Return Value:
//     New 16-bit PWM value
//
// Parameters:
//     None
//
// Description:
//     This function calls the thread to compute a new PWM value.
//------------------------------------------------------------------------------
U16 MTR_pid_compute_newpwm(void)
{
    newpwm = SLR_pwm_duty;

    (void)MTR_pid_compute_thread();

    return newpwm;
}


//------------------------------------------------------------------------------
// U8 MTR_pid_compute_thread(void)
//------------------------------------------------------------------------------
// Return Value:
//     One of Protothread return values:
//     {PT_WAITING, PT_YIELDED, PT_EXITED, PT_ENDED}
//
// Parameters:
//     None
//
// Description:
//     This function implements a thread for the simple velocity form
//     PI control for computing the appropriate PWM value to be applied
//     to the sensorless BLDC motor PWM signal.
//------------------------------------------------------------------------------
static U8 MTR_pid_compute_thread(void)
{
    PT_BEGIN(&pidth);

    while (1)
    {
        PT_WAIT_UNTIL(&pidth, pid_flag);

        pid_flag = 0;

        // 50 SYSCLKs
        curspeed = SLR_motor_current_rpm;

        // 168 SYSCLKs
        tmpval.S32 =  (S32)(U32)Kp * ((S32)(U32)lastspeed - curspeed);

        PT_YIELD(&pidth);

        // 222 SYSCLKs
        tmpval.S32 += (S32)(U32)Ki * ((S32)(U32)SLW_target_rpm - curspeed);

        PT_YIELD(&pidth);

        // 6 SYSCLKs
        lastspeed = curspeed;

        // Scale up by 256 - we assume final number does not overflow
        tmpval.S32 >>= 8;

        if (tmpval.S32 < 0)
        {
            tmpval.S32 = -tmpval.S32;
            if (tmpval.U32 > LIMIT_STEP_DELTA)
            {
                newpwm = LIMIT_STEP_DELTA;
            }
            else
            {
                newpwm = tmpval.U16[LSB];
            }

            // Limit the PWM value to avoid overflow
            if (SLR_pwm_duty < newpwm)
            {
                // Rollover will occur - limit PWM to zero
                newpwm = 0;
            }
            else
            {
                newpwm = SLR_pwm_duty - newpwm;
            }
        }
        else
        {
            if (tmpval.U32 > LIMIT_STEP_DELTA)
            {
                newpwm = LIMIT_STEP_DELTA;
            }
            else
            {
                newpwm = tmpval.U16[LSB];
            }
            newpwm += SLR_pwm_duty;
            if (newpwm < SLR_pwm_duty)
            {
                // Roll over occurred - adjust the PWM to max limit
                newpwm = 0xffff;
            }
        }
        pid_done = 1;
    }
    PT_END(&pidth);
}
#endif



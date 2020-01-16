//-----------------------------------------------------------------------------
// motor.h
//-----------------------------------------------------------------------------
// Copyright 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Created on: Apr 18, 2013
//     Author: sgghang
//-----------------------------------------------------------------------------

#ifndef MOTOR_H
#define MOTOR_H

#include "bldcdk.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

// State machine code
#define MOTOR_STOPPED   0
#define MOTOR_RUNNING   2

#define REPEATED_NUM_OF_TIMER_FOR_STARTUP   4
#define NO_OF_STARTUP_DELAY                 3     // start_delay[] size

#define SET_TIMER0_HIGH_PRIORITY() \
do                                 \
{                                  \
    IP |= 0x02;                    \
} while(0)                         \

#define SET_PCA0_HIGH_PRIORITY()   \
do                                 \
{                                  \
    EIP1 |= 0x08;                  \
} while(0)                         \

#define SET_TIMER0_NORMAL_PRIORITY() \
do                                   \
{                                    \
    IP &= ~0x02;                     \
} while(0)                           \

#define SET_PCA0_NORMAL_PRIORITY()   \
do                                   \
{                                    \
    EIP1 &= ~0x08;                   \
} while(0)                           \


#define SET_CPT0_HIGH_PRIORITY()     \
do                                   \
{                                    \
    EIP1 |= 0x20;                   \
} while(0)                           \

#define SET_CPT0_NORMAL_PRIORITY()   \
do                                   \
{                                    \
    EIP1 &= ~0x20;                   \
} while(0)                           \


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern const       U16 code start_delay[];
extern const       U8 code compMux[];
extern SEG_XDATA   UU16   timer_stamp0;
extern SEG_XDATA   UU16   timer_stamp1;
extern SEG_XDATA   UU32   time_per_roate;
extern SEG_XDATA   U8     num_commutation;

extern SEG_IDATA   U8  SLR_motor_state;
extern SEG_DATA    U8  commutation_index;

extern SEG_DATA    UU32 prev_zc_time;
extern SEG_DATA    UU32 zc_time;
extern SEG_DATA    UU32 zc_commutate_time;
extern SEG_DATA    U8  hall_status;
extern SEG_DATA    U8 expected_next_pattern;
extern SEG_DATA    U8 open_phase;

extern SEG_XDATA   U16 SLR_motor_current_rpm;
extern SEG_DATA    U16 SLW_target_rpm;
extern SEG_XDATA   U16 SLW_target_pwm_duty;
extern SEG_XDATA   U16 phase_current[];
extern SEG_DATA    U16 SLR_pwm_duty;
extern SEG_XDATA   U8 miss_match_count;
extern SEG_XDATA   U16 current_amp_ref;
extern SEG_XDATA   U8 pid_calc_interval;
extern SEG_DATA    U8   zc_count;
extern SEG_DATA    UU32   time_per_rotate;
extern SEG_XDATA   U16 SLW_acceleration_step_size;
extern SEG_XDATA   U16 SLW_deceleration_step_size;
extern SEG_XDATA   U16 error_event_time;
extern             UU16 SLR_minimum_duty;
extern             U16 SLR_minimum_rpm;

extern bit speed_updated;
extern bit SLW_user_direction;
extern bit OC_flag;
extern bit pid_flag;
extern bit test_flag;
extern bit rising_bemf;
extern bit SLR_motor_stalled;
// status for application level.
extern bit SLW_rpm_updated;
extern bit SLW_pwm_updated;

// OP amp bias voltage to be subtracted after measuring phase current.
extern SEG_XDATA U16 SLR_motor_current;

// number of oc debounce at register. 0x05
extern SEG_XDATA U8 SLW_oc_debounce;
// reg. 0x04. over current threshold.
extern SEG_XDATA U8 SLW_current_limit;

#ifdef FEATURE_MEAS_VMDC
extern SEG_XDATA U16 SLR_motor_voltage;
#endif

extern SEG_XDATA U16 SLW_motor_max_rpm;



//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// SL_MTR_init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
//-----------------------------------------------------------------------------
void SL_MTR_init(void);

//-----------------------------------------------------------------------------
// SL_MTR_motor
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description  :
//      This function should be called regularly at application level.
//
//-----------------------------------------------------------------------------
void SL_MTR_motor(void);

//-----------------------------------------------------------------------------
// MTR_check_motor_spinning
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description:
//  Check motor running and direction status before alignment.
//  This should be called just before starting.
//-----------------------------------------------------------------------------
void MTR_check_motor_spinning(void);

//-----------------------------------------------------------------------------
// SL_MTR_start_motor
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description  :
//      Application level would call this to start motor.
//      If motor is not in state MOTOR_STOPPED, it will just return.
//      It will check motor spinning status for fast startup and if there is
//      enough BEMF found, it skips the blind spinning stage.
//      After staring motor, SLR_motor_state would be MOTOR_RUNNING.
//-----------------------------------------------------------------------------
void SL_MTR_start_motor(void);

//-----------------------------------------------------------------------------
// MTR_pre_commutation
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// To minimize overhead in timer 0 interrupt routine, prepare all skip/port
// registers before needing.
//-----------------------------------------------------------------------------
void MTR_pre_commutation(void);

//-----------------------------------------------------------------------------
// MTR_hyper_commutate
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This is to commutate to hyper drive mode.
//-----------------------------------------------------------------------------
void MTR_hyper_commutate(void);

//-----------------------------------------------------------------------------
// MTR_commutate
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This is to commutate next phase.
// open_phase is to refer to current open phase in order to make simple life
// when it need. If use commutation_index, we should refer to motor_direction
// bit as well to know current open phase.
//-----------------------------------------------------------------------------
void MTR_commutate(void);

//-----------------------------------------------------------------------------
// MTR_start_spinning
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Alignment and blind spinning.
// At the end, this will de-energize all phases to spin freely.
// This function has used comparator clear function in PCA.
// In order to cut current as expected, comparator has to have fastest response.
// Small delay in response make significant current surge. (a few us delay is
// pretty significant in 24kHz(42us period) PWM frequency)
// Please refer to application note for more details.
//-----------------------------------------------------------------------------
void MTR_start_spinning(void);

//-----------------------------------------------------------------------------
// MTR_do_quickstart
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description:
//      Assumed that motor already running freely with enough bemf.
//      To Detect rising and falling edge of bemf and calculate next
//      expected commutation time and appropriate commutation index.
//-----------------------------------------------------------------------------
void MTR_do_quickstart(void);

//-----------------------------------------------------------------------------
// SL_MTR_stop_motor
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description  :
//  Stop running motor and de-energize phases only when SLR_motor_state is
//  MOTOR_RUNNING, otherwise it will just return.
//  After stopping motor, disable FG signal(TP308) and prepare comparator
//  to detect BEMF for fast restart.
//  Following variables will be
//  SLR_motor_state = MOTOR_STOPPED.
//  SLR_motor_current_rpm = 0
//  SLR_rpm_updated = 1
//-----------------------------------------------------------------------------
void SL_MTR_stop_motor(void);

//-----------------------------------------------------------------------------
// MTR_calculate_motor_rpm
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description:
//      This function should be called regularly.
//      Speed would be updated after one mechanical rotation.
//-----------------------------------------------------------------------------
void MTR_calculate_motor_rpm(void);

//-----------------------------------------------------------------------------
// MTR_save_zero_crossing_time
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
//  This function should be called in the interrupt routine or
//  It should be protected by MTR_ATOMIC_ACCESS_START() MTR_ATOMIC_ACCESS_END()
//-----------------------------------------------------------------------------
void MTR_save_zero_crossing_time(void);


//-----------------------------------------------------------------------------
// MTR_calculate_pid
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description:
//      If BLDC_RD_RPM_OR_PWM is PWM_PARAMETER,
//      - target speed controlled by SLW_target_pwm_duty variables.
//      If BLDC_RD_RPM_OR_PWM is RPM_PARAMETER,
//      - target speed controlled by SLW_target_rpm variables.
//-----------------------------------------------------------------------------
void MTR_calculate_pid(void);

//-----------------------------------------------------------------------------
// SL_MTR_time
//-----------------------------------------------------------------------------
//
// Return Value : xtime
//      high 16bit of virtual 32bit timer.
//      time unit would be 2.7ms.
// Parameters   : None
//
// Description:
//
//      Get the high 16 bit of virtual 32bit timer.
//-----------------------------------------------------------------------------
U16 SL_MTR_time(void);

//-----------------------------------------------------------------------------
// SL_MTR_change_num_poles
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//      (U8) poles : number of poles. It must be even number
// Description:
//
//-----------------------------------------------------------------------------
void SL_MTR_change_num_poles(U8 poles);

//-----------------------------------------------------------------------------
// MTR_process_adc
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description: Process ADC results and perform filtering or offset adjustment
//
//-----------------------------------------------------------------------------
static void MTR_process_adc(void);

//-----------------------------------------------------------------------------
// MTR_process_adc
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description: Handle motor error events and timing
//
//-----------------------------------------------------------------------------
static void MTR_process_errors(void);

//-----------------------------------------------------------------------------
// SL_MTR_GET_32BIT_TIME
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : x, must be UU32 type variable.
//
// Description:
//
//      Get the 32 bit virtual timer value.
//      This should be called inside interrupt routine.
//      static U8 ovf must be declared at user interrupt routine.
//-----------------------------------------------------------------------------
#define SL_MTR_GET_32BIT_TIME(x)                                \
    do {                                                        \
        EA = 0;                                                 \
        x.UU16[LSB].U8[LSB] = PCA0L;                            \
        x.UU16[LSB].U8[MSB] = PCA0H;                            \
        ovf = CF;                                               \
        x.U16[MSB] = user_timer;                                \
        EA = 1;                                                 \
        if (ovf && ( (x.UU16[LSB].U8[MSB] & 0x80) == 0 ) )      \
        {                                                       \
            x.U16[MSB]++;                                       \
        }                                                       \
    } while(0)


#define MTR_set_direction(b)    do { motor_direction = (b); } while (0)

#endif



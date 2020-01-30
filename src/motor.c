//-----------------------------------------------------------------------------
// motor.c
//-----------------------------------------------------------------------------
// Copyright 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Created on: Apr 18, 2013
//     Author: sgghang
//
// Adapted on: Jan 16, 2020
// Author: a92862
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

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// Table for blind starting. Additional values are calculated in the function to
// save code space.
// Please refer to AN794 for more details derived.
// IMPORTANT.  Please update _LAST_INIT_START_DELAY in BLDC_RD_System.h if
//             start_delay[2] is modified
const U16 code start_delay[NO_OF_STARTUP_DELAY] = {50000, 30902, 23863};

// sequence of open phase : C, A, B, C, A, B ==> CW
// comparator mux : CMP0MX = CMX0N[7:4]:CMX0P[3:0]
// Zero crossing : Falling edge of comparator output.
// Falling bemf : CP0N = VMY(P0.3) CP0P=VMA,VMB, VMC (P0.0~P0.2)
// Rising bemf  : CP0N = VMA,VMB,VMC(P0.0~P0.2) CP0P=VMY(P0.3)
// comMux[0~5] is for Falling bemf
// comMux[6~11] is for Rising bemf
const U8 code compMux[12] = {
    (CPMUX_Y << 4) | CPMUX_C,
    (CPMUX_Y << 4) | CPMUX_A,
    (CPMUX_Y << 4) | CPMUX_B,
    (CPMUX_Y << 4) | CPMUX_C,
    (CPMUX_Y << 4) | CPMUX_A,
    (CPMUX_Y << 4) | CPMUX_B,
    (CPMUX_C << 4)| CPMUX_Y,
    (CPMUX_A << 4)| CPMUX_Y,
    (CPMUX_B << 4)| CPMUX_Y,
    (CPMUX_C << 4)| CPMUX_Y,
    (CPMUX_A << 4)| CPMUX_Y,
    (CPMUX_B << 4)| CPMUX_Y
};

#if BLDC_RD_PWM_METHOD == H_BRIDGE_HIGH_SIDE_PWM
// high side pwm scheme with Si8230
// 1111 0111 : P1.3(PWM),P1.4(L): Phase A->B energized, C open
// 0111 1111 : P1.4(L),P1.7(PWM): Phase C->B energized, A open
// 0111 1111 : P1.2(L),P1.7(PWM): Phase C->A energized, B open
// 1101 1111 : P1.2(L),P1.5(PWM): Phase B->A energized, C open
// 1101 1111 : P1.5(PWM),P1.6(L): Phase B->C energized, A open
// 1111 0111 : P1.3(PWM),P1.6(L): Phase A->C energized, B open
const U8 code skipPattern[6] = {
    ~MOTDRV_AH_MASK, ~MOTDRV_CH_MASK, ~MOTDRV_CH_MASK,
    ~MOTDRV_BH_MASK, ~MOTDRV_BH_MASK, ~MOTDRV_AH_MASK
};
#elif BLDC_RD_PWM_METHOD == H_BRIDGE_LOW_SIDE_PWM
// low side pwm scheme with Si8230
// 1110 1111 : P1.3(H),P1.4(PWM): Phase A->B energized, C open
// 1110 1111 : P1.4(PWM),P1.7(H): Phase C->B energized, A open
// 1111 1011 : P1.2(PWM),P1.7(H): Phase C->A energized, B open
// 1111 1011 : P1.2(PWM),P1.5(H): Phase B->A energized, C open
// 1011 1111 : P1.5(H),P1.6(PWM): Phase B->C energized, A open
// 1011 1111 : P1.3(H),P1.6(PWM): Phase A->C energized, B open
const U8 code skipPattern[6] = {
    ~MOTDRV_BL_MASK, ~MOTDRV_BL_MASK, ~MOTDRV_AL_MASK,
    ~MOTDRV_AL_MASK, ~MOTDRV_CL_MASK, ~MOTDRV_CL_MASK
};
#elif BLDC_RD_PWM_METHOD == H_BRIDGE_MIXED_MODE_PWM
// mixed mode pwm scheme with Si8230
// Falling BEMF (rising_bemf = 1 at setup time):
// 1110 1111 : P1.3(H),P1.4(PWM): Phase A->B energized, C open
// 1110 1111 : P1.4(PWM),P1.7(H): Phase C->B energized, A open
// 1111 1011 : P1.2(PWM),P1.7(H): Phase C->A energized, B open
// 1111 1011 : P1.2(PWM),P1.5(H): Phase B->A energized, C open
// 1011 1111 : P1.5(H),P1.6(PWM): Phase B->C energized, A open
// 1011 1111 : P1.3(H),P1.6(PWM): Phase A->C energized, B open

// Rising BEMF (rising_bemf = 0 at setup time):
// 1111 0111 : P1.3(PWM),P1.4(L): Phase A->B energized, C open
// 0111 1111 : P1.4(L),P1.7(PWM): Phase C->B energized, A open
// 0111 1111 : P1.2(L),P1.7(PWM): Phase C->A energized, B open
// 1101 1111 : P1.2(L),P1.5(PWM): Phase B->A energized, C open
// 1101 1111 : P1.5(PWM),P1.6(L): Phase B->C energized, A open
// 1111 0111 : P1.3(PWM),P1.6(L): Phase A->C energized, B open

// First index is the rising_bemf flag
const U8 code skipPattern[2][6] = {
        {
            ~MOTDRV_AH_MASK, ~MOTDRV_CH_MASK, ~MOTDRV_CH_MASK,
            ~MOTDRV_BH_MASK, ~MOTDRV_BH_MASK, ~MOTDRV_AH_MASK
        },
        {
            ~MOTDRV_BL_MASK, ~MOTDRV_BL_MASK, ~MOTDRV_AL_MASK,
            ~MOTDRV_AL_MASK, ~MOTDRV_CL_MASK, ~MOTDRV_CL_MASK
        },
};
#endif

const U8 code freeterminal[6] = {
    FILTERED_C_MASK, FILTERED_A_MASK, FILTERED_B_MASK,
    FILTERED_C_MASK, FILTERED_A_MASK, FILTERED_B_MASK
};
const U8 code active_gpio[6] = {
    MOTDRV_HILO_ON(A, B),
    MOTDRV_HILO_ON(C, B),
    MOTDRV_HILO_ON(C, A),
    MOTDRV_HILO_ON(B, A),
    MOTDRV_HILO_ON(B, C),
    MOTDRV_HILO_ON(A, C)
};


#ifdef FEATURE_HYPERDRIVE
// First 6 [0-5] are for falling BEMF, and last 6 [6-11] are for rising BEMF
const U8 code hyperactive_gpio[12] = {
    MOTDRV_HILOLO_ON(A, B, C),
    MOTDRV_HILOLO_ON(C, B, A),
    MOTDRV_HILOLO_ON(C, A, B),
    MOTDRV_HILOLO_ON(B, A, C),
    MOTDRV_HILOLO_ON(B, C, A),
    MOTDRV_HILOLO_ON(A, C, B),

    MOTDRV_HILOHI_ON(A, B, C),
    MOTDRV_HILOHI_ON(C, B, A),
    MOTDRV_HILOHI_ON(C, A, B),
    MOTDRV_HILOHI_ON(B, A, C),
    MOTDRV_HILOHI_ON(B, C, A),
    MOTDRV_HILOHI_ON(A, C, B)
};
#endif

SEG_IDATA U8      SLR_motor_state = MOTOR_STOPPED;
// expected next commutation index value
SEG_DATA  U8      commutation_index;

// prepare skip register value in advance so at the end we need just
// assignment code in order to minimize overhead.
SEG_XDATA U8      pre_p0skip;
SEG_XDATA U8      pre_p1skip;
SEG_XDATA U8      pre_port1;

// prev_zc_time, zc_time are time stamp of 32bit virtual timer to calculate
// period of commutation.
// prev_zc_time is used in startup function to find out initial commutation
// time.
SEG_DATA UU32     prev_zc_time;
SEG_DATA UU32     zc_time;
SEG_DATA UU32     zc_commutate_time;
SEG_XDATA U32 zc_total_per_mech_rotation;

// measuring commutation time (one electrical rotate)
SEG_DATA UU32 time_per_rotate;
// for fast phase reference.
SEG_DATA U8       open_phase;

// timer for hyperdrive mode
#ifdef FEATURE_HYPERDRIVE
SEG_DATA U8       hyperdrive_speed;
SEG_XDATA UU16    xhyp_speed;
SEG_XDATA U16     hrem_time;
#endif

// calculated current motor running rpm
SEG_XDATA U16     SLR_motor_current_rpm;
#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
// user requested (BY POT, PWM input or PC) target motor rpm.
SEG_DATA U16      SLW_target_rpm;
#endif

// user requested (BY POT, PWM input or PC) target motor pwm.
SEG_XDATA U16     SLW_target_pwm_duty;
// current pwm duty cycle
SEG_DATA U16      SLR_pwm_duty;
// interval for pi calculation or POT(or PWM input) adc.
// unit : 65536*(1/24.5M) = about 2.7ms.
// value should be (2^n - 1), for fast process in interrupt routine.
SEG_XDATA U8      pid_calc_interval;
SEG_DATA U8       zc_count;
#if BLDC_RD_RPM_OR_PWM == PWM_PARAMETER
SEG_XDATA U16     SLW_acceleration_step_size;
SEG_XDATA U16     SLW_deceleration_step_size;
#endif
// BLDC motor numbe of poles. (must be even number)
SEG_XDATA U8      num_poles;
SEG_XDATA U8      zc_total_count;
// time stamp when over current or stall happens.
SEG_XDATA U16     error_event_time;
// minimum duty cycle for startup and running.
UU16 SLR_minimum_duty;
U16  SLR_minimum_rpm;

bit speed_updated;             // every electrical rotation will set this bit.
bit SLW_user_direction;        // CW = 0, default. to avoid motor direction
bit pid_flag = 0;              // time for pid calculation
bit SLW_rpm_updated;           // status for application level.
bit SLW_pwm_updated;           // status for application level.
bit SLR_motor_stalled;
bit last_motor_stalled;
bit handle_motor_error;
static bit rising_bemf;
static bit motor_spinning;
static bit motor_direction;    // CW = 0, default
#ifdef FEATURE_PID_TUNE_FUNCTION
extern bit tune_in_progress;
#endif

// OP amp bias voltage to be subtracted after measuring phase current.
static SEG_XDATA U16    opamp_offset;
SEG_XDATA U16 SLR_motor_current;

// Additional motor current to accumulate to perform further averaging
// Number of additional accumulation to improve result
#define CURRENT_SHIFT_ACC   (4)
#define CURRENT_ACC         (1 << CURRENT_SHIFT_ACC)
static SEG_XDATA U32    accum_motor_current;
static SEG_XDATA U8     accum_motor_current_count;
static SEG_XDATA U8     average_motor_current_adc;

// number of oc debounce at register. 0x05
SEG_XDATA U8 SLW_oc_debounce;
// reg. 0x04. over current threshold.
SEG_XDATA U8 SLW_current_limit;

// To monitor stall condition by delta_V, Delta_I
static SEG_XDATA U16    prev_rpm;
static SEG_XDATA U16    prev_I;
static SEG_XDATA U8     stall_check_count;

#ifdef FEATURE_MEAS_VMDC
SEG_XDATA U16 SLR_motor_voltage;
#endif

SEG_XDATA U16 SLW_motor_max_rpm;


static void MTR_process_adc(void);
static void MTR_process_errors(void);

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
void SL_MTR_init(void)
{
	PCA_initialize_pca();
    TMR_init_timer3();
    TMR_init_timer0();
	COMP_initialize_comp();
    PCA0L = 0;
    PCA0H = 0;
    // start PWM
    PCA_enable_pwm();
    PCA0CN0_CR = 1;
    TMR2CN0_TR2 = 1;
    pid_calc_interval = DEFAULT_PID_INTERVAL;
    motor_spinning = 0;
    // To avoid garbage value, SLR_minimum_duty will be calculated in smart way.
    // However, if user stopped motor by reset_pin and started it immediately,
    // there is no chance to calculate SLR_minimum_duty because code will
    // skip the MTR_start_spinning().
    SLR_minimum_duty.U16 = (MAX_USER_PWM_VALUE>>2);
    handle_motor_error = 0;
    SLW_user_direction = motor_direction = 0;
    SLR_motor_state = MOTOR_STOPPED;
    SLR_motor_current_rpm = 0;
    // updated and calculated SLR_minimum_rpm at SL_MTR_change_num_poles()
    //num_poles = BLDC_RD_NUM_POLES;
    //SLR_minimum_rpm = (U16)((60UL*1.2*2*SYSCLK)/(65536*2*6*SPEED_UNIT))/num_poles;
#ifndef BUILD_FOR_PROTOCOL
    SLW_oc_debounce = MOTOR_OC_MAX_DEBOUNCE;
    SLW_current_limit = MOTOR_OC;
    SLW_motor_max_rpm = MOTOR_MAX_RPM;
#endif

}


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
void SL_MTR_motor(void)
{
    MTR_process_errors();
    MTR_process_adc();
    if( MOTOR_STOPPED == SLR_motor_state)
    {
        // Initialize variables while in stopped state
        accum_motor_current_count = CURRENT_ACC;
        accum_motor_current = 0;
        average_motor_current_adc = 0;
        SLR_motor_current = 0;
    }
    else if( MOTOR_RUNNING == SLR_motor_state)
    {
        MTR_calculate_motor_rpm();

#ifndef FEATURE_PID_TUNE_FUNCTION
        MTR_calculate_pid();
#endif
        // SL_MTR_stop_motor() will change state
    }

    // Clear the adc_flags for current and vmotor
    adc_flags &= ~( (0x01<<ADC_MUX_CURRENT)
#ifdef FEATURE_MEAS_VMDC
            | (0x01 << ADC_MUX_VMOTOR)
#endif
        );
}

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
void SL_MTR_start_motor(void)
{
    if ((MOTOR_STOPPED != SLR_motor_state) || SLR_motor_stalled )
    {
        return;
    }
    // respect user selected direction
    MTR_set_direction(SLW_user_direction);
    MTR_check_motor_spinning();
    if(!motor_spinning)
    {
        MTR_start_spinning();
    }
    MTR_do_quickstart();

    // Initial stall condition has been checked in the MTR_do_quickstart().
    // Initial startup - take a longer time to check for potential stall
    stall_check_count = 255;
    last_motor_stalled = 0;
    handle_motor_error = 0;

    // change status to running mode
    SLR_motor_state = MOTOR_RUNNING;
}


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
void SL_MTR_stop_motor(void)
{
    bit saved_ea;

    // disable timer0 interrupt
    IE_ET0 = 0;
    // disable CPT0 interrupt
    EIE1 &= ~0x20;
    if(MOTOR_RUNNING != SLR_motor_state)
    {
        return;
    }
    // PCA, timer2 will be running continuously.
    // default pin status.
    // keep P1.0, P1.1 latch status
    MTR_ATOMIC_ACCESS_START();
    P1 = ((P1 & ~MOTDRV_ALL_MASK) | MOTDRV_ALL_OFF) | ~MOTDRV_ALL_MASK;
    P1SKIP |= MOTDRV_ALL_MASK;
    MTR_ATOMIC_ACCESS_END();
    // stop timer 3 and clear flag
    TMR3CN0 &= ~0x84;

    // keep tracking one phase of bemf
    CMP0MX = (CPMUX_Y << 4) | CPMUX_A;

    SLR_motor_state = MOTOR_STOPPED;
    SLR_motor_current_rpm = 0;
    SLW_rpm_updated = 1;                // for apps level status.
    SLR_pwm_duty = SLR_minimum_duty.U16;
    PCA_change_duty_cycle(SLR_pwm_duty);
}

//-----------------------------------------------------------------------------
// MTR_check_motor_spinning
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Description:
//  Check motor running and direction status before alignment.
//  This should be called just before MTR_start_spinning & MTR_do_quickstart.
//-----------------------------------------------------------------------------
void MTR_check_motor_spinning(void)
{
    U8 t1, t2;       // t1: old, t2: new
    U8 gap;
    bit rising_zc_done, cptout;


    CMP0CN0 = 0x8F;      // 20mV Hysteresis
    // clear flags
    CMP0CN0 &= ~0x30;
    // set P0.0~P0.3 to pure analog pins because we need to bemf to determine
    // current motor rotation using comparator.
    P0MDIN &= ~FILTERED_ALLPINS_MASK;
    t1 = (U8)SL_MTR_time();
    t2 = t1;
    motor_spinning = 0;
    rising_zc_done = 0;
    do
    {
        t2 = (U8)SL_MTR_time();
        gap = t2 - t1;
        // rising zc found ?
        if( (0 == rising_zc_done) && (CMP0CN0 & 0x20) )
        {
            t1 = t2;
            rising_zc_done = 1;
            CMP0CN0 &= ~0x30;
        }
        // falling zc found ?
        else if( (1 == rising_zc_done) && (CMP0CN0 & 0x10) )
        {
            goto check_direction;
        }
    } while(gap < VALID_TIME_FOR_QUICK_START);
    // No BEMF or too low.
    goto exit_check_spinning;

check_direction:
    // CP0- = C, CP0+ = B
    // B>C ==> 1(CCW), B<C ==> 0(CW)
    CMP0MX = (CPMUX_C << 4) | (CPMUX_B);
    // cmp = (B>C)?1:0; B>C ==>CCW(1), B<C ==>CW(0)
    // just considering comparator response time.
    t1 = 0;
    while(--t1);
    cptout = ((CMP0CN0 & 0x40) == 0x40);
    if( cptout == motor_direction)
    {
        motor_spinning = 1;
    }
    else
    {
        motor_spinning = 0;
    }
exit_check_spinning:
    CMP0CN0 = 0x80;      // 0mV Hysteresis
    P0MDIN |= FILTERED_ALLPINS_MASK;     // set P0.0~P0.3 to digital pins
}

#if (BLDC_RD_PWM_METHOD == H_BRIDGE_HIGH_SIDE_PWM) || \
    (BLDC_RD_PWM_METHOD == H_BRIDGE_LOW_SIDE_PWM) || \
    (BLDC_RD_PWM_METHOD == H_BRIDGE_MIXED_MODE_PWM)
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
void MTR_pre_commutation(void)
{
    pre_port1 = active_gpio[commutation_index];
#if BLDC_RD_PWM_METHOD == H_BRIDGE_MIXED_MODE_PWM
    pre_p1skip = skipPattern[rising_bemf][commutation_index];
#else
    pre_p1skip = skipPattern[commutation_index];
#endif
    if (rising_bemf)
    {
        // rising_bemf is the current state - we need to prepare for the new
        // state where BEMF is falling

        pre_p0skip = DEFAULT_P0SKIP ^ VIRTUAL_NEUTRAL;
    }
    else
    {
        pre_p0skip = DEFAULT_P0SKIP ^ freeterminal[commutation_index];
    }
}

#ifdef FEATURE_HYPERDRIVE
//-----------------------------------------------------------------------------
// MTR_hyper_commutate
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This is to commutate to hyper drive mode.
//-----------------------------------------------------------------------------
void MTR_hyper_commutate(void)
{
    if (rising_bemf)
    {
        P1 = (P1 & ~MOTDRV_ALL_MASK) | hyperactive_gpio[open_phase+6];
    }
    else
    {
        P1 = (P1 & ~MOTDRV_ALL_MASK) | hyperactive_gpio[open_phase];
    }
}
#endif

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
void MTR_commutate(void)
{
    // don't disable cross bar. if disable it, other peripherals would be
    // disabled such as UART.

    pre_port1 |= (P1 & ~MOTDRV_ALL_MASK);
    // keep latch of P1.0,P1.1
    P1 = pre_port1 | ~MOTDRV_ALL_MASK;
    P0SKIP = pre_p0skip;
    P1SKIP = pre_p1skip;
    // open_phase = commutation_index
    open_phase = commutation_index;

    if (0 == motor_direction)
    {
        commutation_index ++;
        if( commutation_index > 5)
            commutation_index = 0;
    }
    // CCW
    else
    {
        commutation_index--;
        if( commutation_index > 5)
            commutation_index = 5;
    }
}
#endif


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
void MTR_start_spinning(void)
{
    U8 idx, tmp;
    bit saved_ea;
    U32 SEG_XDATA SIG_k_1;    // sum(n=0,...,K-1)
    U32 SEG_XDATA SIG_k;      // sum(n=0,...,K)
    U16 T_k;                  // T_k

    // set initial polarity before staring.
    PCA_set_initial_polarity();

    zc_time.U32 = 0L;
    prev_zc_time.U32 = 0L;

    CMP0MD = 0x00;                  // fastest response time of comparator.

    SIG_k = 0L;

    // configure comparator 0 input pins
    // If I_MEAS > LDO(1.8V), comparator output is low.
    CMP0MX = (CPT0MX_IMEASURE<<4) | CPT0MX_LDO;

    // settling time
    TMR_delay_timer3(1);

    // 50% of duty cycle.
    SLR_pwm_duty = (MAX_USER_PWM_VALUE>>1);
    PCA_change_duty_cycle(SLR_pwm_duty);

    // start comparator clear function
    PCA0CLR = (1<<MOTPWM_CHANNEL);

    // Initial commutation
    commutation_index = 0;
    rising_bemf = 0;

    // alignment
    MTR_ATOMIC_ACCESS_START();
    MTR_pre_commutation();
    MTR_commutate();
    MTR_ATOMIC_ACCESS_END();
    tmp = LOCKING_TIME_FOR_ALIGNMENT;
    while(--tmp > 0)
    {
        TMR_delay_timer3(3);
    }

    // blind spinning, increase speed linearly. - refer to AN
    commutation_index = 0;
    idx = 0;
    do
    {
        // commutation
        MTR_ATOMIC_ACCESS_START();
        MTR_pre_commutation();
        MTR_commutate();
        MTR_ATOMIC_ACCESS_END();

        if( idx < NO_OF_STARTUP_DELAY)
        {
            T_k = start_delay[idx++];
        }
        else
        {
            // calculate next commutation time. approximately calculated
            // to avoid using sqrt() function.
            // calculate T[k+1]
            T_k = SIG_k_1*T_k/SIG_k;
        }
        SIG_k_1 = SIG_k;
        SIG_k += T_k;

        TMR3RL = -T_k;
        TMR3 = TMR3RL;
        TMR3CN0 &= ~0x80;
        // Start Timer 3
        TMR3CN0 = 0x04;
        //prev_acc_time = acc_time;
        tmp = 0;
        CMP0CN0 &= ~0x30;
        while(tmp < REPEATED_NUM_OF_TIMER_FOR_STARTUP)
        {
            if( TMR3CN0 & 0x80 )
            {
                TMR3CN0 &= ~0x80;
                tmp++;
            }
            if((CMP0CN0 & 0x10) == 0x10)
            {
                CMP0CN0 &= ~0x10;
                MTR_ATOMIC_ACCESS_START();
                SLR_minimum_duty.U8[LSB] = PCA0L;
                SLR_minimum_duty.U8[MSB] = PCA0H;
                MTR_ATOMIC_ACCESS_END();
            }
        }
        // Stop Timer 3
        TMR3CN0 &= ~0x04;
        if (T_k < LAST_DELAY_LIMIT)
        {
            break;
        }
    } while(1);

    SLR_minimum_duty.U16 = SLR_minimum_duty.U16<<(16-PWM_RESOLUTION);
    SLR_minimum_duty.U16 -= 32768;
    if(SLR_minimum_duty.U16 >= 32768)
    {
        SLR_minimum_duty.U16 = 0x7FFF;
    }
    SLR_pwm_duty =  SLR_minimum_duty.U16;

    // disable comparator clear function and disconnect all signals.
    CMP0MX = 0xFF;              // default ..
    PCA0CLR = 0;

    // stop energizing and let motor run freely to detect BEMF
    P1 = ((P1 & ~MOTDRV_ALL_MASK) | MOTDRV_ALL_OFF) | ~MOTDRV_ALL_MASK;
    P1SKIP |= MOTDRV_ALL_MASK;
}

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
void MTR_do_quickstart(void)
{
    bit saved_ea, is_1st_crossing;
    UU32 ticks;

    // -----------------------------------------------------------------------
    // start investigating of zero crossing

    // Make filtered pins analog - we do not need the blanking signal
    P0MDIN &= ~FILTERED_ALLPINS_MASK;

    // select PHASE A rising and falling edge to determine next expected
    // commutation index and time based on selected spin direction.
    CMP0MX = (CPMUX_Y << 4) | CPMUX_A;
    // 1ms is more than enough for comparator settling time.
    TMR_delay_timer3(1);
    // clear comparator flags
    CMP0CN0 &= ~0x30;

    // wait max 32 ms loop.
    TMR3RL = 0;
    TMR3 = (U16)0;
    // Start Timer 3
    TMR3CN0 = 0x04;

    // prev_zc_time.U16[LSB] : used to record zero crossing event at startup
    // to save ram usage.
    prev_zc_time.U16[LSB] = 0;
    is_1st_crossing = 1;
    while( !(TMR3CN0 & 0x80) )
    {
        if (CMP0CN0 & 0x30)
        {
            // This is the first event
            if ( is_1st_crossing )
            {
                // stop timer 3 and restart to record next event.
                MTR_ATOMIC_ACCESS_START();
                TMR3CN0 &= ~0x04;
                TMR3 = (U16)0;
                TMR3CN0 = 0x04;
                MTR_ATOMIC_ACCESS_END();
            }
            else
            {
                // second event, get the time period.
                MTR_ATOMIC_ACCESS_START();
                TMR3CN0 &= ~0x04;
                prev_zc_time.U16[LSB] = TMR3;
                // to measure offset (overhead)
                TMR3 = 0;
                TMR3CN0 = 0x04;
                MTR_ATOMIC_ACCESS_END();
            }


            // BEMF Falling, comparator output Falling edge
            if (CMP0CN0 & 0x10)
            {
                // CW
                if( 0 == motor_direction)
                {
                    open_phase = 1;         // current open phase
                    commutation_index = 2;  // next commutation index
                }
                // CCW
                else
                {
                    open_phase = 4;         // current open phase
                    commutation_index = 3;  // next commutation index
                }
                rising_bemf = 0;
            }
            // BEMF rising, comparator output rising edge
            else if(CMP0CN0 & 0x20)
            {
                // CW
                if( 0 == motor_direction)
                {
                    open_phase = 4;         // current open phase
                    commutation_index = 5;  // next commutation index
                }
                // CCW
                else
                {
                    open_phase = 1;         // current open phase
                    commutation_index = 0;  // next commutation index
                }
                rising_bemf = 1;
            }
            // clear flags
            CMP0CN0 &= ~0x30;

            if( is_1st_crossing )
            {
                is_1st_crossing = 0;
            }
            else
            {
                break;
            }
        }
    }
    // if motor stalled, do not energize it.
    if(TMR3CN0 & 0x80)
    {
        SLR_motor_stalled = 1;
        return;
    }
    SLR_motor_stalled = 0;

    // calculate next expected commutation time from current zero-crossing.
    // ticks is time from rising to falling zero-crossing time of phase A.
    // That is time for 180 deg.
    // Desired period for 30 deg is:
    // period = prev_zc_time.U16[LSB]*TIMER3_PRESCALER(=12)/6
    //ticks.U32 = (U32)TIMER3_PRESCALER * prev_zc_time.U16[LSB] / 6;
    // This is for 30deg rotate(zero-crossing to commutation)
    ticks.U32 = 2UL * prev_zc_time.U16[LSB];

    // zc_commutate_time is a period between zero crossing.
    // NOTE : need to initialize zc_commute_time, zc_time, pre_zc_time
    // to do exponential filtering in the comparator interrupt routine.
    MTR_save_zero_crossing_time();
    zc_commutate_time.U32 = 2UL * ticks.U32;

    MTR_ATOMIC_ACCESS_START();
    // Stop Timer 3(DIV_12) and compensate overhead
    TMR3CN0 &= ~0x04;
    ticks.U32 -= ((U32)TIMER3_PRESCALER * TMR3);
    repeated_timer0 = 0;
    timer0_next.U16 = -ticks.U16[LSB];

    TCON_TR0 = 0;
    TL0 = timer0_next.U8[LSB];
    TH0 = timer0_next.U8[MSB];
    // start timer 0 as soon as possible
    TCON_TR0 = 1;
    TCON_TF0 = 0;
    MTR_ATOMIC_ACCESS_END();

    // zero crossing detected. next event is TIMER0_COMMUTATION.
    // Prepare skip register and timer0_state value.
    MTR_pre_commutation();
    timer0_state = TIMER0_COMMUTATION;
    zc_total_count = (num_poles >> 1) * 6;
    zc_count = zc_total_count;

    // just in case, disable comparator interrupt
    CMP0CN0 &= ~0x30;
    CMP0MD &= ~0x30;

    SET_TIMER0_HIGH_PRIORITY();
    // enable timer0 interrupt
    IE_ET0 = 1;
    // enable comparator0 interrupt.
    // now comparator interrupt enabled/disabled by CMP0MD.
    EIE1 |= 0x20;

    // for initial rpm calculation
    // at this moment, atomic access not required (motor not under running)
    // for time_per_rotate
    // to avoid current surge by big jump in pwm_duty.
    SLR_motor_current_rpm = (U32)(60UL*SYSCLK*2/6/SPEED_UNIT)/((U32)num_poles*zc_commutate_time.U32);
    speed_updated = 0;
    SLW_rpm_updated = 0;
    time_per_rotate.U32 = 0UL;
    zc_total_per_mech_rotation = 0UL;
    // initiate pwm filtering according to rising/falling bemf.
    SLW_pwm_updated = 0;
    PCA_change_duty_cycle(SLR_pwm_duty);
#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
    // initialize PID variables.
    MTR_pid_init();
#endif
}

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
//      This will set the SLW_rpm_updated variable for application level.
//-----------------------------------------------------------------------------
void MTR_calculate_motor_rpm(void)
{
    SEG_XDATA U32 t0;
    bit saved_ea;

    // -----------------------------------------------------------------------
    // calculate current rpm based on at least 1 mechanical rotation
    // rpm = 60 / (ttr * PAIR_OF_POLES * 6)
    if (speed_updated)
    {
        speed_updated = 0;
        MTR_ATOMIC_ACCESS_START();
        t0 = zc_total_per_mech_rotation;
        MTR_ATOMIC_ACCESS_END();
        SLR_motor_current_rpm = (U16)((SYSCLK*60/SPEED_UNIT)/t0);
        // status for application level
        SLW_rpm_updated = 1;
    }
}


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
void MTR_save_zero_crossing_time(void)
{
    static U8 ovf;

    prev_zc_time.U32 = zc_time.U32;
    IE_EA = 0;
    zc_time.UU16[LSB].U8[LSB] = PCA0L;
    zc_time.UU16[LSB].U8[MSB] = PCA0H;
    ovf = PCA0CN0_CF;
    zc_time.U16[MSB] = user_timer;
    IE_EA = 1;

    if (ovf && ( (zc_time.UU16[LSB].U8[MSB] & 0x80) == 0 ) )
    {
        zc_time.U16[MSB]++;
    }
}

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
void MTR_calculate_pid(void)
{
#ifdef FEATURE_HYPERDRIVE
    U16 scaled_pwm;
    U16 xspeed;
#endif

#if BLDC_RD_RPM_OR_PWM == RPM_PARAMETER
    // MTR_pid_compute_newpwm() should be called regularly. This function
    // has PT_YIELD(due to pretty high cpu load to calculate it) and
    // will clear pid_flag.
    SLR_pwm_duty = MTR_pid_compute_newpwm();

#elif BLDC_RD_RPM_OR_PWM == PWM_PARAMETER
    U16 pwmnew;

    if(!pid_flag)
    {
        return;
    }
    pid_flag = 0;

    // emulated pid function.
    if( SLW_target_pwm_duty > SLR_pwm_duty)
    {
        pwmnew = SLR_pwm_duty + SLW_acceleration_step_size;
        if(pwmnew < SLR_pwm_duty)
        {
            // This is to handle overflow across 0xffff-0x0000 boundary
            pwmnew = MAX_USER_PWM_VALUE;
        }
        SLR_pwm_duty = pwmnew;
    }
    else if(SLW_target_pwm_duty < SLR_pwm_duty)
    {
        pwmnew = SLR_pwm_duty - SLW_deceleration_step_size;
        if(pwmnew < SLR_minimum_duty.U16)
        {
            // Protect against PWM duty cycle dropping below startup value
            pwmnew = SLR_minimum_duty.U16;
        }
        SLR_pwm_duty = pwmnew;
    }
    pid_done = 1;
#endif

#ifdef FEATURE_HYPERDRIVE

    if (pid_done)
    {
        // Scale up by 10% - we want to reserve the remainder for hyperdrive mode
        scaled_pwm = ((U32)(PWM_SCALE_FACTOR * 65536.0) * SLR_pwm_duty) >> 16;
        if (scaled_pwm < SLR_pwm_duty)
        {
            // This is to handle overflow across 0xffff-0x0000 boundary
            scaled_pwm = MAX_USER_PWM_VALUE;
        }

        // Hyperdrive will start working when scaled_pwm is 100%,
        // or SLR_pwm_duty > (65536.0/PWM_SCALE_FACTOR).
        if ( (scaled_pwm ^ MAX_USER_PWM_VALUE) == 0 )
        {
            // Obtain hyperdrive timing based on fraction of 256
            // larger number implies faster speed [0-3]
            xspeed = ((U16)(SLR_pwm_duty - (U16)(65536.0/PWM_SCALE_FACTOR)) * HYPERDRIVE_FACTOR) >> HYPER_SHIFT;
            if (xhyp_speed.U16 > xspeed)
            {
                xhyp_speed.U16--;
            }
            else if (xhyp_speed.U16 < xspeed)
            {
                xhyp_speed.U16++;
            }
        }
        else
        {
            xhyp_speed.U16 = 0;
        }
        hyperdrive_speed = xhyp_speed.U16 >> (8-HYPER_SHIFT);
        pid_done = 0;
    }
    else
    {
        scaled_pwm = SLR_pwm_duty;
    }
    PCA_change_duty_cycle(scaled_pwm);
#else
    PCA_change_duty_cycle(SLR_pwm_duty);
#endif
}


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
U16 SL_MTR_time(void)
{
    bit saved_ea;
    U16 xtime;

    MTR_ATOMIC_ACCESS_START();
    xtime = user_timer;
    MTR_ATOMIC_ACCESS_END();

    return xtime;
}

//-----------------------------------------------------------------------------
// SL_MTR_change_num_poles
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//      (U8) poles : number of poles. It must be even number
// Description:
//      This function should be called at least one time to calculate
//      SLR_minimum_rpm.
//-----------------------------------------------------------------------------
void SL_MTR_change_num_poles(U8 poles)
{
    num_poles = poles;
    // 1.2 is just head room.
    SLR_minimum_rpm = (U16)((60UL*1.2*2*SYSCLK)/(65536*2*6*SPEED_UNIT))/num_poles;
}

//-----------------------------------------------------------------------------
// MTR_process_adc
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description: Process ADC results and perform filtering or offset adjustment
//
//-----------------------------------------------------------------------------
static void MTR_process_adc(void)
{
    static SEG_XDATA U16 prev_opamp_offset = 0;
    UU16 SEG_XDATA tmp;

    if ( adc_flags & (0x01<<ADC_MUX_CURRENT) )
    {
        // get latest saved phase current in the array
        tmp.U16 = adc_result[ADC_MUX_CURRENT];

        // read op amp bias voltage for future current calculation.
        if( MOTOR_STOPPED == SLR_motor_state )
        {
            opamp_offset = adc_result[ADC_MUX_CURRENT];
            opamp_offset = (prev_opamp_offset>>1) + \
                           ((prev_opamp_offset+opamp_offset)>>2);
            prev_opamp_offset = opamp_offset;
        }
        else
        {
            if( opamp_offset > tmp.U16)
            {
                tmp.U16 = 0;
            }
            else
            {
                tmp.U16 -= opamp_offset;
            }

            accum_motor_current += tmp.U16;
            if (--accum_motor_current_count == 0)
            {
                tmp.U16 = accum_motor_current >> CURRENT_SHIFT_ACC;
                average_motor_current_adc = tmp.U16 >> (ADC_RESOLUTION-8);
                accum_motor_current = 0;
                accum_motor_current_count = CURRENT_ACC;

                //current in 0.01A
                // Only 14bits need for PC side. (2^14 * 0.01A = 163A max value)
                SLR_motor_current = (((U32)tmp.U16 * ROUND_DIV((U32)CURRENT_UNIT * \
                        (U32)ADC_REF_VOLTAGE*2UL*(1 << (16-ADC_RESOLUTION)), \
                                (U32)(OP_AMP_GAIN*LOAD_RESISTOR)))>>16) & 0x3fff;
            }
        }
    }

#ifdef FEATURE_MEAS_VMDC
    if( adc_flags & (0x01<<ADC_MUX_VMOTOR) )
    {
        // get latest saved value in the array
        tmp.U16 = adc_result[ADC_MUX_VMOTOR];

        // unit is 0.01 Volts (10mV)
        SLR_motor_voltage = ( (U32)tmp.U16 * ROUND_DIV(VMOTOR_DIVIDER_R1_PLUS_R2 * 2UL \
                * ADC_REF_VOLTAGE * (1 << (16-ADC_RESOLUTION)), \
                10UL*VMOTOR_DIVIDER_R1)) >> 16;
    }
#endif
}

//-----------------------------------------------------------------------------
// MTR_process_adc
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description: Handle motor error events and timing
//
//-----------------------------------------------------------------------------
static void MTR_process_errors(void)
{
    static U8 oc_count;
    U16 gap;
    U16 delta_I;
    U16 rpm_changes,i_changes;

    if (SLR_motor_stalled && !last_motor_stalled)
    {
        last_motor_stalled = 1;
        error_event_time = SL_MTR_time();
        // signal for error condition
        CLR_FG();
    }

    if (SLR_motor_stalled)
    {
        gap = SL_MTR_time() - error_event_time;
        if (gap > ERROR_EVENT_EXPIRE_TIME)
        {
            last_motor_stalled = 0;
            SLR_motor_stalled = 0;
            SET_FG();
        }
        return;
    }

    if ( adc_flags & (0x01<<ADC_MUX_CURRENT) )
    {
#ifdef FEATURE_OVERCURRENT
        if( SLW_oc_debounce && SLW_current_limit)
        {
            if ( (MOTOR_RUNNING == SLR_motor_state) && \
                    ( average_motor_current_adc >= SLW_current_limit) )
            {
                oc_count++;
                if (oc_count >= SLW_oc_debounce)
                {
                    handle_motor_error = 1;
                }
            }
            else
            {
                oc_count = 0;
            }
        }
#endif

#ifdef FEATURE_RPM_STALL_DETECTION
        if(--stall_check_count == 0)
        {
            // Just check when I(current) was increased and target speed is
            // higher than current speed because load causes decreasing speed.
            // if (Const + (target_speed+Vnew)/2 - Vold) < (k*dI) then, motor is stalled.
            if( (SLR_motor_current > prev_I) && (SLW_target_rpm > SLR_motor_current_rpm))
            {
                delta_I = SLR_motor_current - prev_I;
#if COMPENSATION_CONSTANT_FACTOR == 0
                rpm_changes = ((SLW_target_rpm + SLR_motor_current_rpm)>>1);
#else
                rpm_changes = ((U32)COMPENSATION_CONSTANT_FACTOR * SLW_motor_max_rpm) >> 16;
                rpm_changes += ((SLW_target_rpm + SLR_motor_current_rpm)>>1);
#endif
                rpm_changes = (rpm_changes - prev_rpm);
                i_changes = ((U32)DELTA_CURRENT_FACTOR_K * SLW_motor_max_rpm * delta_I) >> 16;
                // (C + Vnew - Vold) < k*dI ?
                if( rpm_changes < i_changes)
                {
                    handle_motor_error = 1;
                }
            }
            stall_check_count = STALL_CHECK_COUNT;
            prev_rpm = SLR_motor_current_rpm;
            prev_I = SLR_motor_current;
        }
#endif
    }

    if (handle_motor_error)
    {
        SL_MTR_stop_motor();
        SLR_motor_stalled = 1;
        handle_motor_error = 0;
    }

}


//-----------------------------------------------------------------------------
// CMP0_ISR
//-----------------------------------------------------------------------------
//
// CMP0 ISR Content goes here. Remember to clear flag bits:
// CMP0CN0::CPFIF (Comparator Falling-Edge Flag)
// CMP0CN0::CPRIF (Comparator Rising-Edge Flag)
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (CMP0_ISR, CMP0_IRQn)
{
	static UU32 ticks;

	    // below code takes about 25us
	    MTR_save_zero_crossing_time();
	    ticks.U32 = zc_time.U32 - prev_zc_time.U32;

	    //Exponential filter
	    zc_commutate_time.U32 = (zc_commutate_time.U32>>1) +
	            ((zc_commutate_time.U32+ticks.U32)>>2);

	    // for rpm calculation
	    time_per_rotate.U32 += zc_commutate_time.U32;
	    if(--zc_count == 0)
	    {
	        zc_total_per_mech_rotation = time_per_rotate.U32;
	        time_per_rotate.U32 = 0UL;
	        zc_count = zc_total_count;
	        speed_updated = 1;
	    }

	    // time to next commutation.
	    ticks.U32 = (zc_commutate_time.U32 >> 1);
	    if (ticks.U32 > (PHASE_ADVANCE+4))
	    {
	        ticks.U32 -= PHASE_ADVANCE;
	    }
	    else
	    {
	        ticks.U32 = 4;
	    }

	    // expected next commutation time
	    repeated_timer0 = 0;
	    timer0_next.U16 = ticks.U16[LSB];
	    TCON &= ~0x30;
	#ifdef FEATURE_HYPERDRIVE
	    if (hyperdrive_speed)
	    {

	        // additional phase advance = (hrem_time * (hyperdrive_speed/256));
	        // max(hyperdrive_speed) == 255
	        hrem_time = (ticks.U16[LSB] >> HYPER_CMT_SHIFT);
	        ticks.U16[MSB] = (U16)(hrem_time >> 8) * hyperdrive_speed;
	        ticks.U16[MSB] += ((U16)(hrem_time & 0xff) * hyperdrive_speed) >> 8;
	        ticks.U16[LSB] += ticks.U16[MSB];

	        timer0_next.U16 = ticks.U16[LSB];

	        //Re-use MSB of ticks
	        ticks.UU16[MSB].U8[MSB] = 255 - hyperdrive_speed;

	        if (ticks.UU16[MSB].U8[MSB])
	        {
	            // hrem_time = (high(ticks.U16[LSB])*(255-hyperdrive_speed))>>8
	            hrem_time = (U16)ticks.UU16[LSB].U8[MSB] * ticks.UU16[MSB].U8[MSB];
	            hrem_time += ((U16)ticks.UU16[LSB].U8[LSB] * ticks.UU16[MSB].U8[MSB]) >> 8;
	            ticks.U16[LSB] = hrem_time;

	        }
	        else
	        {
	            // We should not energize the 3rd terminal immediately
	            // here as it would trigger a sudden surge in current
	            // best to go through timer interrupt to have a more
	            // gradual increase in current
	            ticks.U16[LSB] = 1;

	        }
	        // hrem_time = (timer0_next + timer0_next*x) * y; (x < 1.0, y < 1.0)
	        // x --> additional phase advance factor
	        hrem_time = timer0_next.U16 - ticks.U16[LSB];
	        timer0_state = TIMER0_START_HYPERDRIVE;

	        timer0_next.U16 >>= 1;  // Half the remaining time for skipping inductive kick
	    }
	    else
	#endif
	    {
	        timer0_state = TIMER0_COMMUTATION;
	    }
	    ticks.U16[LSB] = -ticks.U16[LSB];

	    TL0 = ticks.UU16[LSB].U8[LSB];
	    TH0 = ticks.UU16[LSB].U8[MSB];
	    TCON_TR0 = 1;

	    //disable comparator interrupt
	    CMP0MD = 0x00;
	    CMP0CN0 &= ~0x30;

	#ifdef FEATURE_FG
	    if ((commutation_index == 0) || (commutation_index == 3))
	    {
	        TOGGLE_FG();
	    }
	#endif

	    // pre_calculation to minimize overhead in timer 0 commutation time.
	    MTR_pre_commutation();

	    SET_CPT0_NORMAL_PRIORITY();
	    // This must be last - otherwise, timer interrupt may be executed prematurely.
	    SET_TIMER0_HIGH_PRIORITY();
}


//-----------------------------------------------------------------------------
// TIMER0_ISR
//-----------------------------------------------------------------------------
//
// TIMER0 ISR Content goes here. Remember to clear flag bits:
// TCON::TF0 (Timer 0 Overflow Flag)
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (TIMER0_ISR, TIMER0_IRQn)
{
    static UU16 hyt;
    TCON_TF0 = 0;
here:
    // to avoid any overhead, do critical one first.
    if ( TIMER0_COMMUTATION == timer0_state )
    {
        if( 0 == repeated_timer0 )// it takes about 8 us
        {
            MTR_commutate();

            // Make filtered pins digital - to enable blanking signal
            P0MDIN |= FILTERED_ALLPINS_MASK;

            rising_bemf = ~rising_bemf;
            if(rising_bemf)
            {
            	CMP0MX = compMux[open_phase + 6];
            }
            else
            {
            	CMP0MX = compMux[open_phase];
            }
            repeated_timer0 = 0;
            // 12.5% (7.5deg)
            timer0_next.U16 = -(timer0_next.U16>>2);
            TCON &= ~0x30;
            TL0 = timer0_next.U8[LSB];
            TH0 = timer0_next.U8[MSB];
            TCON_TR0 = 1;
            timer0_state = TIMER0_SKIP_INDUCTIVE_KICK;
            SET_TIMER0_HIGH_PRIORITY();
            SET_CPT0_NORMAL_PRIORITY();
            //disable comparator interrupt
            CMP0CN0 &= ~0x30;
            CMP0CN0 &= ~0x30;
        }
        else
        {
            repeated_timer0--;
            TCON &= ~0x30;
            TL0 = timer0_next.U8[LSB];
            TH0 = timer0_next.U8[MSB];
            TCON_TR0 = 1;
        }
    }
#ifdef FEATURE_HYPERDRIVE
    else if ( TIMER0_START_HYPERDRIVE == timer0_state )
    {
        // Hyperdrive mode
        timer0_state = TIMER0_COMMUTATION;
        TCON &= ~0x30;
        hyt.U8[LSB] = TL0;
        hyt.U8[MSB] = TH0;
        if (hyt.U16 >= hrem_time)
        {
            goto here;
        }
        hyt.U16 -= hrem_time;
        TL0 = hyt.U8[LSB];
        TH0 = hyt.U8[MSB];
        TCON_TR0 = 1;
        MTR_hyper_commutate();
    }
#endif
    else if ( TIMER0_SKIP_INDUCTIVE_KICK == timer0_state )
    {
        // enable comparator interrupt
        // falling edge detection for ZC
    	CMP0CN0 &= ~0x30;
    	CMP0MD = 0x00;
    	CMP0MD = 0x10;

        // waiting for zero crossing...
        // comparator interrupt will detect zero-crossing event.
        repeated_timer0 = 0xFF;
        TCON &= ~0x30;
        TL0 = 0;
        TH0 = 0;
        TCON_TR0 = 1;
        timer0_state = TIMER0_ZERO_DETECTING;
        SET_TIMER0_NORMAL_PRIORITY();
        SET_CPT0_HIGH_PRIORITY();
    }
    else if ( TIMER0_ZERO_DETECTING == timer0_state )
    {
        repeated_timer0--;
        // Could not detect zero crossing long time.
        // Stall
        if (repeated_timer0 < 0xF0)
        {
            handle_motor_error = 1;
        }
    }
}


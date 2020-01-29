//------------------------------------------------------------------------------
// BLDC_RD_System.h
//------------------------------------------------------------------------------
// Copyright (C) 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Description:
//
// This file defines system parameters for BLDC reference design.
//
// Release 0.0 - May 10, 2013 mufayyuz
//    -Initial Revision.
// Adapted Jan 6, 2020 a92862

#ifndef INC_BLDC_RD_SYSTEM_H_
#define INC_BLDC_RD_SYSTEM_H_

#include <SI_EFM8BB1_Register_Enums.h>
//------------------------------------------------------------------------------
// System clock frequency in Hz
//------------------------------------------------------------------------------
#define SYSCLK 24500000UL
#define BAUD_RATE 115200

//------------------------------------------------------------------------------
// Test motor selection
//------------------------------------------------------------------------------
#define BLY172S_24V_4000            0
#define TRAXXAS                     1
#define TURNIGY                     2
#define TEST_MOTOR                  TURNIGY

//=============================================================================
// PLATFORM CONFIGURABLE SECTION
// Users can change parameters in this section to suit their hardware
// implementation
//=============================================================================
// Features to enable/disable.  Comment out to disable
// FEATURE_FG
//   FG is a signal that toggles every half electrical cycle.
//   Commonly used in fan applications
// FEATURE_OVERCURRENT
//   Over current detect feature.  Need to enable this in Design Kit
//   to observe current
// FEATURE_MEAS_VMDC
//   Measure Motor voltage
// FEATURE_MEAS_POT
//   Measure resistance potentiometer
// FEATURE_HYPERDRIVE
//   Enable the hyperdrive feature
// FEATURE_LED0
//   LED functionality - P2.0 on ref design board
// FEATURE_BTN1
//   BTN1 functionality - P2.1 on ref design board

//#define FEATURE_FG
#define FEATURE_OVERCURRENT
#define FEATURE_MEAS_VMDC
#define FEATURE_MEAS_POT
//#define FEATURE_HYPERDRIVE
//#define FEATURE_LED0
#define FEATURE_RPM_STALL_DETECTION
//#define FEATURE_PID_TUNE_FUNCTION

#if BLDC_RD_RPM_PWM_SRC == PWM_SPEED_SOURCE
    #define FEATURE_PWMIN
    #ifdef FEATURE_PWMIN
        #warning "FEATURE_MEAS_POT disabled forcefully because FEATURE_PWMIN"
        #warning "and FEATURE_MEAS_POT are using same pin"
        #warning "FEATURE_RPM_STALL_DETECTION disabled forcefully"
        #warning "because of code space"
        #warning "User can change this at BLDC_RD_System.h if need"
        #warning "according to requirements"
        #undef FEATURE_MEAS_POT
        #undef FEATURE_RPM_STALL_DETECTION
    #endif
#endif

#define FEATURE_BTN0
#define FEATURE_BTN1

#ifdef FEATURE_PID_TUNE_FUNCTION
#warning "FEATURE_OVERCURRENT disabled forcefully."
#undef FEATURE_OVERCURRENT
#warning "FEATURE_MEAS_VMDC disabled forcefully."
#undef FEATURE_MEAS_VMDC
#warning "FEATURE_HYPERDRIVE disabled forcefully."
#undef FEATURE_HYPERDRIVE
#endif

#if BLDC_RD_RPM_OR_PWM == PWM_PARAMETER
#undef FEATURE_RPM_STALL_DETECTION
#endif

// FG not implemented in this design
#define FG_PORT         P2
#define FG_PIN          0

// WARNING - This clashes with POT_ADCMX.
#define PWMIN_PORT      P1
#define PWMIN_BIT       0

// WARNING - This clashes with BTN0
#define LED0_PORT       P1
#define LED0_BIT        1

//-----------------------------------------------------------------------------
// MCU gate driver pins - assumed to be on Port 1
//-----------------------------------------------------------------------------
#define MOTDRV_AL_PIN   2
#define MOTDRV_AH_PIN   3
#define MOTDRV_BL_PIN   4
#define MOTDRV_BH_PIN   5
#define MOTDRV_CL_PIN   6
#define MOTDRV_CH_PIN   7

// Gate drive active level.  0 = active low, 1 = active high
#define MOTDRV_LOW_ACT  1
#define MOTDRV_HIGH_ACT 1
//-----------------------------------------------------------------------------
// MCU connections to filtered A, B, C and Y terminals
// - must be on Port 0.
// - defined with respect to CPT0MX selection assignment
//-----------------------------------------------------------------------------
#define FILTERED_A_PIN          0
#define FILTERED_B_PIN          1
#define FILTERED_C_PIN          2
#define FILTERED_Y_PIN          3


//-----------------------------------------------------------------------------
// Comparator clear function in PCA for startup
// Note: This can be used to limit current during startup.
//       In order to control the limit of current, external voltage reference
//       that connected CP- may be used. In this design, internal LDO(1.8V)
//       is used as a reference.
//-----------------------------------------------------------------------------
#define CPT0MX_IMEASURE     6
#define CPT0MX_LDO          8

//-----------------------------------------------------------------------------
// Current measurement pin assignment as ADC0MX assignment
// DO NOT DEFINE THIS if overcurrent feature is not required.
//-----------------------------------------------------------------------------
#ifdef FEATURE_OVERCURRENT
#define IMEAS_ADCMX         6
#endif

//-----------------------------------------------------------------------------
// Motor voltage measurement pin assignment as ADC0MX assignment
// DO NOT DEFINE THIS if motor voltage measurement feature is not required.
//-----------------------------------------------------------------------------
#ifdef FEATURE_MEAS_VMDC
#define VMDC_ADCMX          7
#endif

//-----------------------------------------------------------------------------
// POT measurement pin assignment as ADC0MX assignment
// Note: This may be used even if PWM input is the speed control
//       method if there is a low-pass filter to obtain the equivalent
//       analog voltage
//-----------------------------------------------------------------------------
#ifdef FEATURE_MEAS_POT
#define POT_ADCMX           8
#endif

//-----------------------------------------------------------------------------
// PCA setup for PWM of motor gate driver.
// NOTE: Channel 0 should always be reserved for the blanking signal
//-----------------------------------------------------------------------------
#define MOTPWM_CHANNEL          1               // CEX1

// Define the default P0SKIP
// - now configured to reserve 2 pins for UART
#define DEFAULT_P0SKIP          (0xCF)          // UART P0.4, P0.5
// Define the default P1SKIP when we do not want MOTOR PWM but want
// some other non-MOTOR PWM
#define DEFAULT_P1SKIP          (0xFF)
// Define the skip all pins for PxMASK
#define SKIP_ALL                (0xFF)


//-----------------------------------------------------------------------------
// Hardware/Motor design configurable parameters
//-----------------------------------------------------------------------------
// PWM_SCALE_FACTOR adjusts the input scale factor by an increased factor
// - this will allow us to use a smaller input number to scale up to
//   100% duty cycle
// - the remainder will be used to compute the HYPERDRIVE mode
#define PWM_SCALE_FACTOR    1.1
#define HYPERDRIVE_FACTOR   (U16)(( PWM_SCALE_FACTOR + ( PWM_SCALE_FACTOR - 1 )\
                                * 0.5 ) / ( PWM_SCALE_FACTOR - 1 ) )

// Hyperdrive speed change rate - 0 to 8.  8 is the fastest change allowed
#define HYPER_SHIFT   8

// Hyperdrive commutation delay shift:  Smaller number implies a longer delay
// - longer delay MAY allow us to reach higher speed faster.
// - but longer delay is potentially more unstable
// - range 1 to 6
#define HYPER_CMT_SHIFT 1


// Define speed unit in number of RPM.  We use 10 because
// the firmware is actually able to support up to 200000 RPM for a
// 2-pole motor.  Using 10 will limit our variable sizes to 16-bits.
#define SPEED_UNIT  10
//=============================================================================
// END OF PLATFORM CONFIGURABLE SECTION
//=============================================================================


//TODO: Put motor parameters here
// ----------------------------------------------------------------------------
// BLY172S-24V-4000 MOTOR
// ----------------------
// RATE V = 24V, 55W, line to line inductance = 1.8mH, line to line R = 1.8ohm
// BEMF votage = 3.72V/kRPM
// @RATED RPM (4000), 24 = 3.72*4 + IR, I = 5A = Imax.
//
// TRAXXAS MOTOR
// -------------
// http://traxxas.com/products/parts/motors/velineon3500motor
//
// TURNINGY MOTOR
// -------------
// Operating Voltage: 7.4~14.8V (2~4S li-poly)
// RPM: 3800kv
// Max Current: 35A
// Max Power: 365W
// Idle Current: 3.5A
// Resistance: 0.018ohms
// Shaft: 3.175mm
// Connector: 3.5mm bullet
// Bolt Hole Spacing: 16mm
// Weight: 80g
// http://www.hobbyking.com
// model: Turnigy_450_Series_3800KV_Brushless_Outrunner
//-----------------------------------------------------------------------------
#if (TEST_MOTOR == TURNIGY) || \
    (TEST_MOTOR == TRAXXAS) || \
    (TEST_MOTOR == BLY172S_24V_4000)

// in case of default TURNIGY motor
#define MOTOR_KV                        3800UL
#define MOTOR_SUPPLY_VDD                12.0       // in V
// MOTOR_KV * MOTOR_SUPPLY_VDD * 1.2 / SPEED_UNIT (20% headroom)
#define MOTOR_MAX_SPEED                 (54720)

// This for constant if BUILD_FOR_PROTOCOL is not enabled or user can change
// this in the GUI.
//Added 20% headroom
#define MOTOR_MAX_RPM                   ((U16)(MOTOR_MAX_SPEED / SPEED_UNIT) )
// unit A
#define MOTOR_MAX_CURRENT               (10)

// 5~10% of MOTOR_MAX_RPM would be enough.
// bigger value give smoother start with more current.
#define STARTUP_EXIT_PERCENT            5
// Please refer to below comments for more detail.
#define DELTA_CURRENT_FACTOR_K_PERCENT              50UL
#define COMPENSATION_CONSTANT_FACTOR_PERCENT        5UL
#endif

//-----------------------------------------------------------------------------
// BOARD CONFIGURATION
//-----------------------------------------------------------------------------
#define OP_AMP_GAIN                   1       // J202,J303 installed
#define LOAD_RESISTOR                 50              // unit in mOhm
#define MOTOR_OC_MAX_DEBOUNCE         5

// divider constant by resister
#define VMOTOR_DIVIDER_R1             115UL      // 1.15k
#define VMOTOR_DIVIDER_R2             976UL      // 9.76k
#define VMOTOR_DIVIDER_R1_PLUS_R2     (VMOTOR_DIVIDER_R1 + VMOTOR_DIVIDER_R2)

//-----------------------------------------------------------------------------
// COMMON MOTOR CONFIGURATION
//-----------------------------------------------------------------------------
// unit is 3 ms. Do not exceed 255
#define LOCKING_TIME_FOR_ALIGNMENT    200


// extension of timer for very low rpm.
#define REPEATED_NUM_OF_TIMER_FOR_STARTUP   4

// Decreasing  SARTUP_EXIT_RPM, it will decrease startup period and back emf
// during blind spinning which will cause motor to stop just after blind spinning.
// Set final speed of blind spinning to exit startup state.
#define STARTUP_EXIT_RPM              (U16)( (U32)STARTUP_EXIT_PERCENT * \
                                      MOTOR_MAX_RPM * SPEED_UNIT / 100 )

// This is based on STARTUP_EXIT_RPM
// clock = SYSCLK / 12, TIMER3_PRESCALER
// commut_time = 60 / (STARTUP_EXIT_RPM * (POLE_PAIRS/2) * 6)
// ticks = commut_time * clock
// LAST_DELAY_LIMIT = ticks / REPEATED_NUM_OF_TIMER_FOR_STARTUP
// LAST_DELAY_LIMIT must not exceed 16 bit value (max 65535).
// @ 2 poles && REPEATED_NUM_OF_TIMER_FOR_STARTUP == 1
// ==> minimum valid STARTUP_EXIT_RPM == 312
// @ 2 poles && REPEATED_NUM_OF_TIMER_FOR_STARTUP == 4
// ==> minimum valid STARTUP_EXIT_RPM == 78
// However, it is not the same as the final blind spinning speed
// to have ENOUGH BACK EMF for the proper startup.
// So user must choose STARTUP_EXIT_PERCENT to get ENOUGH back emf to start
// motor.
#define LAST_DELAY_LIMIT              (U16)((U32)( (2UL*SYSCLK*60/6) / \
                                      (BLDC_RD_NUM_POLES * REPEATED_NUM_OF_TIMER_FOR_STARTUP * 12UL) ) / \
                                      STARTUP_EXIT_RPM )

/*********** This section is for doing sanity check  of the startup **********/
// Initial start of delay table - should be in range of 40000 to 60000
// This variable is used to approximate
#define _LAST_INIT_START_DELAY    23863UL



// MOTOR_MAX_RPM = MOTOR_MAX_SPEED / SPEED_UNIT
// STARTUP_EXIT_RPM = STARTUP_EXIT_PERCENT * MOTOR_MAX_RPM * SPEED_UNIT / 100
#define _LAST_DELAY_LIMIT_            (( (2UL * SYSCLK * 60 / 6) / \
                                      (BLDC_RD_NUM_POLES * REPEATED_NUM_OF_TIMER_FOR_STARTUP * 12UL) ) / \
                                      ( STARTUP_EXIT_PERCENT * \
                                      ( (MOTOR_MAX_SPEED / SPEED_UNIT) ) \
                                      * SPEED_UNIT / 100 ) )
#if _LAST_DELAY_LIMIT_ >= _LAST_INIT_START_DELAY
#error "LAST_DELAY_LIMIT value exceed 16bit"
#error "Please increase STARTUP_EXIT_PERCENT or REPEATED_NUM_OF_TIMER_FOR_STARTUP"
#error "in BLDC_RD_System.h"
#endif
/******************** End sanity check  of the startup ************************/

#define ACCELERATION_STEP_SIZE     ((MAX_USER_PWM_VALUE>>(PWM_RESOLUTION-1))+1)
#define DECELERATION_STEP_SIZE     ((MAX_USER_PWM_VALUE>>(PWM_RESOLUTION-1))+1)

#define PI_PROPORTIONAL_GAIN        13000
#define PI_INTEGRAL_GAIN            400

// Constant for quick start
// unit is 2.7ms, do not exceed 30ms
#define VALID_TIME_FOR_QUICK_START                10
// waiting till stop inertia rotation
#define TIME_FOR_STOPPING_INERTIA_ROTATION        100

// Considering cpu overhead time to commutate in time at VERY HIGH commutation
// frequency.
// At comparator interrupt routine, time to enable TR0 is about 20us
// (due to a lot of 32bit calculation). To consider other
// overhead(5us), 25us is selected as a PHASE_ADVANCE.
// If PWM frequency changed, the 60UL should be changed.
#define PHASE_ADVANCE_US            25

// Define PHASE_ADVANCE in units of SYSCLK ticks
#define PHASE_ADVANCE               (U16)ROUND_DIV( SYSCLK * PHASE_ADVANCE_US,\
                                    1000000)

// 1000->0.001A, 100-> 0.01A
#define CURRENT_UNIT                100
// Defined : MSB 8bit of full scale ADC
#define MOTOR_OC_1    ((( ( MOTOR_MAX_CURRENT * LOAD_RESISTOR * \
                      OP_AMP_GAIN) * (MAX_ADC_VALUE + 1) ) / 2 \
                      / ADC_REF_VOLTAGE))
#define MOTOR_OC      ( (U8)( (MOTOR_OC_1)/(1<<(ADC_RESOLUTION - 8)) + 0.5 ) )

// Once over current detected or motor stalled, need to wait user action.
// Wait for at least ERROR_EVENT_EXPIRE_TIME period.
// unit is 2.7ms, Ex: 5000*2.7 = 13.5 Sec.
#define ERROR_EVENT_EXPIRE_TIME                   5000

// time of PID calculation interval (it is related with pid variables)
// Unit is 2.7ms, This value should be (2^n-1)
// Ex. if value is 15, (15+1)*2.7 is the interval.
#define DEFAULT_PID_INTERVAL                    (15)

// Frequency of POT adc interval
// Unit is 2.7ms, This value should be (2^n-1)
// Ex. if value is 15, (15+1)*2.7 is the interval.
#define DEFAULT_SPEED_ADC_INTERVAL              (7)

// Unit is 2.7ms, This value should be (2^n-1)
#define RPM_REPORT_PERIOD                       (3)
//-----------------------------------------------------------------------------
// Configuration for stall detection
//-----------------------------------------------------------------------------
// Basic idea is to observe delta rpm / delta current. Once holding the rotor,
// although target speed is same, pid control loop will try to increase speed
// by increasing the current.
// Stall condition is (KONST + (target_speed + Rpm_new)/2 - Rpm_previous)
// < K * delta_I
// KONST = COMPENSATION_CONSTANT_FACTOR(in %) * motor_max_rpm / 100
// K = DELTA_CURRENT_FACTOR_K(in %) * (motor_max_rpm/max_current) / 100
// Larger value in DELTAL_CURRENT_FACTOR_K or Smaller Value in
// COMPENSATION_CONSTANT_FACTOR cause more sensitive in detecting stall.
// STALL_CHECK_COUNT is to determine period of sampling for delta rpm,
// delta current. Unit is about 10ms. 100*10ms = 1 sec.

// unit is in % then scale up by 65536 then divide by MOTOR_MAX_CURRENT
// in units of CURRENT_UNIT
#define DELTA_CURRENT_FACTOR_K   (DELTA_CURRENT_FACTOR_K_PERCENT * 65536UL / \
                                 (100UL * CURRENT_UNIT * MOTOR_MAX_CURRENT))
// unit is in % then scale up by 65536
#define COMPENSATION_CONSTANT_FACTOR    (COMPENSATION_CONSTANT_FACTOR_PERCENT *\
                                        65536UL / 100)
#define STALL_CHECK_COUNT                   100

//-----------------------------------------------------------------------------
// Derived masks values - DO NOT CHANGE
//-----------------------------------------------------------------------------

// Generic defines for port configuration
#define PxMDOUT(x)  PxMDOUT_(x)
#define PxMDOUT_(x) x ## MDOUT
#define PxMDIN(x)   PxMDIN_(x)
#define PxMDIN_(x)  x ## MDIN
#define PxSKIP(x)   PxSKIP_(x)
#define PxSKIP_(x)  x ## SKIP
#define PxMASK(x)   PxMASK_(x)
#define PxMASK_(x)  x ## MASK
#define PxMAT(x)    PxMAT_(x)
#define PxMAT_(x)  x ## MAT


#if BLDC_RD_PWM_METHOD == H_BRIDGE_HIGH_SIDE_PWM
#define MOTDRV_PWM_ACT      MOTDRV_HIGH_ACT
#elif BLDC_RD_PWM_METHOD == H_BRIDGE_LOW_SIDE_PWM
#define MOTDRV_PWM_ACT      MOTDRV_LOW_ACT
#elif BLDC_RD_PWM_METHOD == H_BRIDGE_MIXED_MODE_PWM
#define MOTDRV_PWM_ACT      MOTDRV_LOW_ACT
#endif

#define MOTDRV_MASK(a)      (1 << (MOTDRV_ ## a ## _PIN))
#define MOTDRV_AL_MASK      MOTDRV_MASK(AL)
#define MOTDRV_AH_MASK      MOTDRV_MASK(AH)
#define MOTDRV_BL_MASK      MOTDRV_MASK(BL)
#define MOTDRV_BH_MASK      MOTDRV_MASK(BH)
#define MOTDRV_CL_MASK      MOTDRV_MASK(CL)
#define MOTDRV_CH_MASK      MOTDRV_MASK(CH)

#define MOTDRV_LOWSIDE_MASK   (MOTDRV_AL_MASK | MOTDRV_BL_MASK | MOTDRV_CL_MASK)
#define MOTDRV_HIGHSIDE_MASK  (MOTDRV_AH_MASK | MOTDRV_BH_MASK | MOTDRV_CH_MASK)
#define MOTDRV_ALL_MASK     (MOTDRV_LOWSIDE_MASK | MOTDRV_HIGHSIDE_MASK)

#define MOTDRV_LOFF(a)      ((1^MOTDRV_LOW_ACT) << (MOTDRV_ ## a ## L_PIN))
#define MOTDRV_HOFF(a)      ((1^MOTDRV_HIGH_ACT) << (MOTDRV_ ## a ## H_PIN))
#define MOTDRV_AL_OFF       MOTDRV_LOFF(A)
#define MOTDRV_AH_OFF       MOTDRV_HOFF(A)
#define MOTDRV_BL_OFF       MOTDRV_LOFF(B)
#define MOTDRV_BH_OFF       MOTDRV_HOFF(B)
#define MOTDRV_CL_OFF       MOTDRV_LOFF(C)
#define MOTDRV_CH_OFF       MOTDRV_HOFF(C)

#define MOTDRV_ATERM_OFF    (MOTDRV_AL_OFF | MOTDRV_AH_OFF)
#define MOTDRV_BTERM_OFF    (MOTDRV_BL_OFF | MOTDRV_BH_OFF)
#define MOTDRV_CTERM_OFF    (MOTDRV_CL_OFF | MOTDRV_CH_OFF)
#define MOTDRV_ALL_OFF      (MOTDRV_ATERM_OFF | MOTDRV_BTERM_OFF | \
                            MOTDRV_CTERM_OFF)

#define MOTDRV_LON(a)       ((MOTDRV_LOW_ACT) << (MOTDRV_ ## a ## L_PIN))
#define MOTDRV_HON(a)       ((MOTDRV_HIGH_ACT) << (MOTDRV_ ## a ## H_PIN))
#define MOTDRV_AL_ON        MOTDRV_LON(A)
#define MOTDRV_AH_ON        MOTDRV_HON(A)
#define MOTDRV_BL_ON        MOTDRV_LON(B)
#define MOTDRV_BH_ON        MOTDRV_HON(B)
#define MOTDRV_CL_ON        MOTDRV_LON(C)
#define MOTDRV_CH_ON        MOTDRV_HON(C)

#define MOTDRV_HILO_ON(hi, lo)  ((MOTDRV_ALL_OFF & ~(MOTDRV_MASK(hi ## H) | \
                                MOTDRV_MASK(lo ## L))) | MOTDRV_HON(hi) | \
                                MOTDRV_LON(lo))
#define MOTDRV_HILOHI_ON(hi, lo, hi1)   ((MOTDRV_ALL_OFF & \
        ~(MOTDRV_MASK(hi ## H) | MOTDRV_MASK(lo ## L) | MOTDRV_MASK(hi1 ## H)))\
        | MOTDRV_HON(hi) | MOTDRV_LON(lo) | MOTDRV_HON(hi1))
#define MOTDRV_HILOLO_ON(hi, lo, lo1)   ((MOTDRV_ALL_OFF & \
        ~(MOTDRV_MASK(hi ## H) | MOTDRV_MASK(lo ## L) | MOTDRV_MASK(lo1 ## L)))\
        | MOTDRV_HON(hi) | MOTDRV_LON(lo) | MOTDRV_LON(lo1))

#define FILTERED_A_MASK         (1 << (FILTERED_A_PIN))
#define FILTERED_B_MASK         (1 << (FILTERED_B_PIN))
#define FILTERED_C_MASK         (1 << (FILTERED_C_PIN))
#define VIRTUAL_NEUTRAL         (1 << (FILTERED_Y_PIN))

#define FILTERED_ALLPINS_MASK   (FILTERED_A_MASK | FILTERED_B_MASK | FILTERED_C_MASK | VIRTUAL_NEUTRAL)

#define CPMUX_A         FILTERED_A_PIN
#define CPMUX_B         FILTERED_B_PIN
#define CPMUX_C         FILTERED_C_PIN
#define CPMUX_Y         FILTERED_Y_PIN


// Defines related to MOTPWM_CHANNEL
#define MOTPWM_CCF                  PCA0CN0_CCF1
#define MOTPWM_PCA0CPM              PCA0CPM1
#define MOTPWM_PCA0CPH              PCA0CPH1
#define MOTPWM_PCA0CPL              PCA0CPL1
#define LOW_DUTY_POLARITY           (MOTDRV_PWM_ACT << MOTPWM_CHANNEL)
#define HIGH_DUTY_POLARITY          ((1^MOTDRV_PWM_ACT) << MOTPWM_CHANNEL)


// Define BTN0
#ifdef FEATURE_BTN0
SI_SBIT(BTN0, SFR_P2, 1);
#define IS_BTN0_PRESSED()	(BTN0 == 0)
#else
#define CONFIG_BTN0()
#define IS_BTN0_PRESSED()   (0)
#endif

// Define BTN1
#ifdef FEATURE_BTN1
SI_SBIT(BTN1, SFR_P1, 1);	// Define BTN1
#define IS_BTN1_PRESSED()   (BTN1 == 1)
#else
#define CONFIG_BTN1()
#define IS_BTN1_PRESSED()   (0)
#endif


#ifdef FEATURE_FG
SBIT(FG, FG_PORT, FG_PIN);
#define FG_MDOUT    PxMDOUT(FG_PORT)
#define CONFIG_FG() do { FG_MDOUT |= 1 << (FG_PIN); } while (0)
#define SET_FG()    do { FG = 1; } while (0)
#define CLR_FG()    do { FG = 0; } while (0)
#define TOGGLE_FG() do { FG ^= 1; } while (0)
#else
#define CONFIG_FG()
#define SET_FG()
#define CLR_FG()
#define TOGGLE_FG()
#endif

#ifdef FEATURE_PWMIN
SBIT(PWMIN, PWMIN_PORT, PWMIN_BIT);
#define PWMIN_MDOUT    PxMDOUT(BTN0_PORT)
#define PWMIN_MASK     PxMASK(PWMIN_PORT)
#define PWMIN_MAT      PxMAT(PWMIN_PORT)
#define CONFIG_PWMIN()       do { PWMIN_MDOUT &= ~(1 << (PWMIN_BIT)); \
                                  PWMIN_MASK = (1<<PWMIN_BIT); \
                                  PWMIN_MAT = (1<<PWMIN_BIT);} while (0)
#define IS_PWMIN_LOW()   (PWMIN == 0)
#else
#define CONFIG_PWMIN()
#define IS_PWMIN_LOW()   (0)
#endif


#ifdef FEATURE_LED0
SBIT(LED0, LED0_PORT, LED0_BIT);
#define LED0_MDOUT  PxMDOUT(LED0_PORT)

#define LED0_ON()   do { LED0 = 0; } while (0)
#define LED0_OFF()  do { LED0 = 1; } while (0)
#define LED0_TOGGLE()   do { LED0 ^= 1; } while (0)

// We don't need to do anything special configure for LED because it works even
// if not configured as push-pull because it is active-low
#define CONFIG_LED0()
#else

#define LED0_ON()
#define LED0_OFF()
#define LED0_TOGGLE()

#define CONFIG_LED0()

#endif

// For analog signals.
typedef enum _PxMDIN_init {
    PxMDIN_INIT_VAL = 0xffff
         & ~(1 << CPT0MX_IMEASURE)
#if defined(FEATURE_OVERCURRENT)
         & ~(1 << IMEAS_ADCMX)
#endif
#if defined(FEATURE_MEAS_VMDC)
         & ~(1 << VMDC_ADCMX)
#endif
#if defined(FEATURE_MEAS_POT)
         & ~(1 << POT_ADCMX)
#endif
} PxMDIN_init;


#endif /* INC_BLDC_RD_SYSTEM_H_ */

//------------------------------------------------------------------------------
// End Of File
//------------------------------------------------------------------------------

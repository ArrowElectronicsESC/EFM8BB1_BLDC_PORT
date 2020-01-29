//------------------------------------------------------------------------------
// BLDC_RD_Build_Params.h
//------------------------------------------------------------------------------
// Copyright (C) 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Description:
//
// This file defines some parameters that are changed at build time configure
// the system for a particular operation.
//
// Release 0.0 - May 10, 2013 mufayyuz
//    -Initial Revision.
// Adapted Jan 6, 2020 a92862

#ifndef INC_BLDC_RD_BUILD_PARAMS_H_
#define INC_BLDC_RD_BUILD_PARAMS_H_


//------------------------------------------------------------------------------
// Build-time settings for 8-bit registers
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Which parameter to be controlled by selected source (see BLDC_RD_RPM_PWM_SRC)
// 0: Target Speed
// 1: Target PWM
// related with reg 0x03,Bit3 and 0x3C, 0x3E
//------------------------------------------------------------------------------
#define RPM_PARAMETER                       0
#define PWM_PARAMETER                       1
#define BLDC_RD_RPM_OR_PWM                  RPM_PARAMETER

//------------------------------------------------------------------------------
// Source of target Speed or target PWM (selected by BLDC_RD_RPM_OR_PWM above)
// 1: External PWM
// 2: ADC input (POT)
// related with reg 0x03, Bit0-1
//------------------------------------------------------------------------------
#define PWM_SPEED_SOURCE                   0
#define POT_SPEED_SOURCE                   1
#define BLDC_RD_RPM_PWM_SRC                POT_SPEED_SOURCE

//------------------------------------------------------------------------------
// Select PWM method
// 0: High-side
// 1: Low-side
//------------------------------------------------------------------------------
#define H_BRIDGE_HIGH_SIDE_PWM          0
#define H_BRIDGE_LOW_SIDE_PWM           1
#define H_BRIDGE_MIXED_MODE_PWM         2
#define BLDC_RD_PWM_METHOD              H_BRIDGE_MIXED_MODE_PWM

//------------------------------------------------------------------------------
// Select commutation method
// 0: Count-down
// 2: Hall sensor
//------------------------------------------------------------------------------
#define COMMUTATION_BY_COUNTDOWN            0
#define COMMUTATION_BY_HALL                 1
#define BLDC_RD_COMMUT_METHOD               COMMUTATION_BY_COUNTDOWN

//------------------------------------------------------------------------------
// Number of poles of selected motor (see selected motor's specifications). If
// motor specifications mention 'pairs of poles' then use twice the value.
// BLDC_RD_NUM_POLES should always be an even number.
//------------------------------------------------------------------------------
#define BLDC_RD_NUM_POLES       6

//------------------------------------------------------------------------------
// Build-time settings for 16-bit registers
//------------------------------------------------------------------------------
//None at the moment


//------------------------------------------------------------------------------
// Build-time setting for communication protocol
//------------------------------------------------------------------------------
#define BUILD_FOR_PROTOCOL

#ifdef BUILD_FOR_PROTOCOL

//------------------------------------------------------------------------------
// Build-time setting for communication interface
//------------------------------------------------------------------------------
#define BUILD_FOR_UART

//Other possible serial interfaces but not supported.
//#define BUILD_FOR_SPI
//#define BUILD_FOR_I2C

//------------------------------------------------------------------------------
// Build-time settings for debugging
//------------------------------------------------------------------------------
//#define BUILD_FOR_DEBUG

#endif //BUILD_FOR_PROTOCOL


//------------------------------------------------------------------------------
// APP_MOTOR_MIN_RPM setting.
// There is the limitation for time measuring due to 16 bit timer.
// The minimum rpm is caused by this 16 bit timer limitation.
//------------------------------------------------------------------------------
#ifdef BUILD_FOR_PROTOCOL   // User can change num_poles thru GUI

// APP_MOTOR_MIN_RPM would be changed dynamically according to register value
// of 0x06(number of pole pairs)
#define APP_MOTOR_MIN_RPM           SLR_minimum_rpm

#else                       // User can change BLDC_RD_NUM_POLES at this file.

// This default value is based on TURNIGY motor. in RPM
// At this moment, SPEED_UNIT(in BLDC_RD_System.h) not defined.
#define _MOTOR_MIN_RPM           1000
// _MOTOR_MIN_RPM value would be evaluated by compiler.
// This is based on period from ZC to commutation time.
// Added 20% head room.
// 60 / [(65536*2/SYSCLK)*6*(BLDC_RD_NUM_POLES/2)] * 1.2
#define _ABS_MOTOR_MIN_RPM       ((60UL*24500000UL*2*12)/(65536UL*6*2*BLDC_RD_NUM_POLES*10))
#if _MOTOR_MIN_RPM < _ABS_MOTOR_MIN_RPM
#error "_MOTOR_MIN_RPM is too low, please increase the value of _MOTOR_MIN_RPM"
#error "_MOTOR_MIN_RPM is defined at BLDC_RD_Build_Params.h"
#endif

#define APP_MOTOR_MIN_RPM   (U16)(_MOTOR_MIN_RPM / SPEED_UNIT)
#endif

//------------------------------------------------------------------------------
// Generate compile-time error for conflicting build configurations
//------------------------------------------------------------------------------
#ifndef BUILD_FOR_PROTOCOL
#warning "Building without Protocol."
#endif //BUILD_FOR_PROTOCOL

/*
#if BLDC_RD_COMMUT_METHOD != _BLDC_RD_COMMUT_METHOD

#if BLDC_RD_COMMUT_METHOD == 0
#error "Mismatched project used: BLDC_RD_COMMUT_METHOD is defined for sensorless operation  \
        but this project is designed for sensored operation.  Please open the \
        f85x_bldc.wsp sensorless project to build the code."
#else
#error "Mismatched project used: BLDC_RD_COMMUT_METHOD is defined for sensored operation  \
        but this project is designed for sensorless operation.  Please open the \
        f85x_bldc_hall.wsp sensored project to build the code."
#endif

#endif
*/


#endif /* INC_BLDC_RD_BUILD_PARAMS_H_ */

//------------------------------------------------------------------------------
// End Of File
//------------------------------------------------------------------------------

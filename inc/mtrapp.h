/*
 * mtrapp.h
 *
 *  Created on: Jan 20, 2020
 *      Author: a92862
 */

#ifndef INC_MTRAPP_H_
#define INC_MTRAPP_H_

//-----------------------------------------------------------------------------
// MTRAPP_init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// initialization for application level.
//-----------------------------------------------------------------------------
void MTRAPP_init();

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
void MTRAPP_task(void);


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
void MTRAPP_read_direction(void);

//-----------------------------------------------------------------------------
// MTRAPP_read_target_speed
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None but it will change global variable SLW_target_rpm.
//
//  set SLW_target_rpm or SLW_target_pwm_duty according to build-time parameter.
//  those target values are saved to register accordingly.
//  To reach 100% of target duty cycle and motor_max_rpm in any case,
//  just added small constant to the adc result.
//-----------------------------------------------------------------------------
void MTRAPP_read_target_speed(void);


//-----------------------------------------------------------------------------
// MTRAPP_read_pwm_input
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
//  Assuming the input frequency is 50~100Hz (20ms ~ 10ms)
//-----------------------------------------------------------------------------
void MTRAPP_read_pwm_input(void);

#endif /* INC_MTRAPP_H_ */

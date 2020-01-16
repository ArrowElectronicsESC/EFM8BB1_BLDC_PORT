/*
 * mtrpid.h
 *
 *  Created on: Jan 16, 2020
 *      Author: a92862
 */

#ifndef INC_MTRPID_H_
#define INC_MTRPID_H_


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
void SL_MTR_change_pid_gain(U16 proportion, U16 integral);

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
void MTR_pid_init(void);

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
U16 MTR_pid_compute_newpwm(void);

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
static U8 MTR_pid_compute_thread(void);

extern bit pid_done;



#endif /* INC_MTRPID_H_ */

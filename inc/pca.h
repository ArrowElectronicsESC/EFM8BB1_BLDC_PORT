/*
 * pca.h
 *
 *  Created on: Jan 16, 2020
 *      Author: a92862
 */

#ifndef INC_PCA_H_
#define INC_PCA_H_


//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define PWM_24KHZ               0
#define PWM_12KHZ               1
#define PWM_FREQ                PWM_24KHZ

#if PWM_FREQ == PWM_12KHZ
    #define PWM_RESOLUTION      11
#elif PWM_FREQ == PWM_24KHZ
    #define PWM_RESOLUTION      10
#endif


#define MAX_USER_PWM_VALUE      (0xFFFF)
#define INITIAL_PCA_DUTY        0
#define MAX_PWM_VALUE           (MAX_USER_PWM_VALUE>>(16-PWM_RESOLUTION))


// Limit duty cycle increment/decrement at pid loop to avoid current surge.
// if increase LIMIT_STEP_DELTA then it reach to target speed faster.
#define EFFECTIVE_MIN_PWM_STEP  ( (U32)( ( MAX_USER_PWM_VALUE >> \
                                ( PWM_RESOLUTION ) ) + 1 ) )
#define LIMIT_STEP_DELTA        ((U32)(EFFECTIVE_MIN_PWM_STEP*32))

// Hysteresis for high / low duty cycle. Unit is percent(%).
#define PWM_FILTER_LOWHYS_PERCENT   35
#define PWM_FILTER_HIGHHYS_PERCENT  40

#define PWM_FILTER_LOWHYS   (U16)( (U32)PWM_FILTER_LOWHYS_PERCENT * \
                                    MAX_PWM_VALUE / 100 )
#define PWM_FILTER_HIGHHYS  (U16)( (U32)PWM_FILTER_HIGHHYS_PERCENT * \
                                    MAX_PWM_VALUE / 100 )

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// initialize_pca
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// PWM - 24kHz frequency, 10bit resolution, Edge aligned signal used by default.
//
// to check zero crossing time, virtual 32bit timer established.
// -user_timer(MSB) + PCA0(LSB)
// CEX0 is used for pwm filtering.
// CEX1 is for FET driving.
//-----------------------------------------------------------------------------
void PCA_initialize_pca(void);


//-----------------------------------------------------------------------------
// PCA_set_initial_polarity
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description  :
//-----------------------------------------------------------------------------
void PCA_set_initial_polarity(void);

//-----------------------------------------------------------------------------
// PCA_enable_pwm
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description  :
//-----------------------------------------------------------------------------
void PCA_enable_pwm(void);

//-----------------------------------------------------------------------------
// PCA_disable_pwm
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
// Description  :
//-----------------------------------------------------------------------------
void PCA_disable_pwm(void);

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
//   real max duty cycle is (2^PWM_RESOLUTION - 1).
//   scaling down is required.
//   Actual new pwm will be updated at pca cycling overflow interrupt.
//
//   high/low duty cycle have different scheme of pwm filtering.
//   Please refer to application note.
//-----------------------------------------------------------------------------
void PCA_change_duty_cycle(U16 duty);

// Macros...
#define PCA_CLEAR_PCA_OF()  do {CF=0;}while(0)
#define PCA_WAIT_PCA_OF() do {} while(!CF)

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern U16 user_timer;

#endif /* INC_PCA_H_ */

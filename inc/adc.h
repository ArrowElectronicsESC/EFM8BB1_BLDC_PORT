/*
 * adc.h
 *
 *  Created on: Jan 16, 2020
 *      Author: a92862
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
typedef enum _en_ADC_MX {
    ADC_MUX_CURRENT,
#ifdef FEATURE_MEAS_VMDC
    ADC_MUX_VMOTOR,
#endif
#ifdef FEATURE_MEAS_POT
    ADC_MUX_POT,
#endif
    NUM_ADC_POINTS
};

#define ADC_REF_VOLTAGE     1650        // in mV, 1.65V
#define ADC_RESOLUTION      14
#define MAX_ADC_VALUE       (0xFFFF>>(16-ADC_RESOLUTION))


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern SEG_XDATA U16    adc_result[];
extern SEG_DATA  U8     adc_flags;


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// ADC_initialize_adc
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Burst mode on
// POWERUP(16 SYSCLK)+[C(12 SARCLK)+T(5 SYSCLK)+C+T+C+T....]
// C + T = 64 SYSCLK
// ADC Timing is synchronized with pwm signal to average current.
// 64 accumulation = 64*64 = 4096 SYSCLK = 23.9kHz/4 (PWM frequency / 4)
//-----------------------------------------------------------------------------
void ADC_initialize_adc(void);

//-----------------------------------------------------------------------------
// ADC_task
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
//
// Description  :
//  AD0BUSY initiate conversion.
//  Once conversion is done, adc_flags updated accordingly. The latest result
//  can be found at the adc_result[] in the point of application level.
//  User application should call this function regularly.
//-----------------------------------------------------------------------------
void ADC_task(void);

// Simple macros...
#define ADC_START() do { ADC0CN0_ADBUSY = 1;} while(0)
#define ADC_WAIT_ADC_DONE() do {} while(!ADC0CN0_ADINT)
#define ADC_CLEAR_FLAG() do {ADC0CN0_ADINT=0;}while(0)


#endif /* INC_ADC_H_ */

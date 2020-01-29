//-----------------------------------------------------------------------------
// adc.c
//-----------------------------------------------------------------------------
// Copyright 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Created on: Apr 22, 2013
//     Author: sgghang
// Adapted on: Jan 16, 2020
// Author: a92862
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "bldcdk.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void ADC_initialize_adc(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// ADC mux table for POT(speed input), VMDC, Current
const U8 code adc_mux[NUM_ADC_POINTS] = {
#ifdef FEATURE_OVERCURRENT
    IMEAS_ADCMX,
#endif
#ifdef FEATURE_MEAS_VMDC
    VMDC_ADCMX,
#endif
#ifdef FEATURE_MEAS_POT
    POT_ADCMX
#endif
};

static SEG_IDATA U8 adc_index;
SEG_XDATA U16       adc_result[NUM_ADC_POINTS];
SEG_DATA U8         adc_flags;

//-----------------------------------------------------------------------------
// ADC_initialize_adc
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Burst mode timing consists of the following segments for non-low-mode:
// (A) Start Burst mode Preparation:
//       7 HFO clocks
// (B) Each SAR conversion consists of:
//       1 HFO clock for MS Bit SAR preparation:
//       4*ADTM SAR clocks before conversion
//       11 SAR clocks for conversion
//       1 HFO clock at end of SAR for LS bit latching
//     Sub-Total = number of accumulation x (sum of above)
// (C) Burst mode tracking time BETWEEN SAR conversion:
//        66-ADTK HFO clocks
//     Sub-Total = (number of accumulation - 1) x (sum of above)
// (D) Synchronization: 3 SYSCLK to transfer status into ADINT bit.
//
// ADC Timing is synchronized with pwm signal to calculate average current.
//
// We use ADSC = 4 (i.e. SARCLK = 5 HFOCLKS)
//        ADTM = 0
//        ADTK = 59
//        ADRPT = 5 (64x accumulation)
// Ignoring (D), the total time for 64 accumulation:
//  7 + ( 1 + ((11 + 4*0) * 5) + 1 ) * 64 + (66 - 59) * (64 - 1) = 4096 HFO clocks
// As the PWM frequency is 10-bit, and 1 SYSCLK = 1 HFO clock, the ADC averaging
//  occurs over exactly 4 PWM cycles
//-----------------------------------------------------------------------------
void ADC_initialize_adc(void)
{
	// ADC Ref : Internal 1.65V
	REF0CN = (0x03<<3);

	// 64 accumulation(16bit effectively) and right justified,shifted right
	// by 2bit. The adc result has 14bit resolution.
	ADC0AC = 0x95;

	// SAR clock = 24.5/(1+ADSC) = 4.9MHz
	// ADSC = 4, ADTM = 0, adc0 gain = 0.5
	ADC0CF = 0x20;

	// ADCMBE = 1,  Common mode buffer enable;
	ADC0CN1 = 0x01;

	// Burst mode track time = 7 HFO CLK: ADC0TK = 66 - 7 = 59
	ADC0TK = 59;

	// ADPWR = 8 SYSCLK
	ADC0PWR = 1;

	// Enable ADC, Start of conversion mode = AD0BUSY
	// Enable Burst mode (ADCMBE=1)
	ADC0CN0 = 0xC0;

    // initiate 1st conversion of adc.
    ADC_CLEAR_FLAG();
    adc_index = 0;
    ADC0MX = adc_mux[adc_index];
    ADC0CN0_ADBUSY = 1;
    adc_flags = 0;
}

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
void ADC_task(void)
{
    if(ADC0CN0_ADINT)
    {
        adc_result[adc_index] = ADC0;
        ADC_CLEAR_FLAG();

        adc_flags |= (1<<adc_index);
        adc_index++;
        if( adc_index >= NUM_ADC_POINTS)
        {
            adc_index = 0;
        }
        ADC0MX = adc_mux[adc_index];
        // start conversion
        ADC_START();
    }
}

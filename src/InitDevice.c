//=========================================================
// src/InitDevice.c: generated by Hardware Configurator
//
// This file will be regenerated when saving a document.
// leave the sections inside the "$[...]" comment tags alone
// or they will be overwritten!
//=========================================================

// USER INCLUDES
#include <SI_EFM8BB1_Register_Enums.h>
#include "InitDevice.h"

// USER PROTOTYPES
// USER FUNCTIONS

// $[Library Includes]
// [Library Includes]$

//==============================================================================
// enter_DefaultMode_from_RESET
//==============================================================================
extern void enter_DefaultMode_from_RESET(void) {
	// $[Config Calls]
	WDT_0_enter_DefaultMode_from_RESET();
	PORTS_0_enter_DefaultMode_from_RESET();
	PORTS_1_enter_DefaultMode_from_RESET();
	PORTS_2_enter_DefaultMode_from_RESET();
	PBCFG_0_enter_DefaultMode_from_RESET();
	ADC_0_enter_DefaultMode_from_RESET();
	CMP_0_enter_DefaultMode_from_RESET();
	CLOCK_0_enter_DefaultMode_from_RESET();
	TIMER01_0_enter_DefaultMode_from_RESET();
	TIMER16_2_enter_DefaultMode_from_RESET();
	TIMER16_3_enter_DefaultMode_from_RESET();
	TIMER_SETUP_0_enter_DefaultMode_from_RESET();
	PCA_0_enter_DefaultMode_from_RESET();
	PCACH_0_enter_DefaultMode_from_RESET();
	UART_0_enter_DefaultMode_from_RESET();
	INTERRUPT_0_enter_DefaultMode_from_RESET();
	// [Config Calls]$

}

//================================================================================
// WDT_0_enter_DefaultMode_from_RESET
//================================================================================
extern void WDT_0_enter_DefaultMode_from_RESET(void) {
	// $[Watchdog Timer Init Variable Declarations]
	uint32_t i;
	bool ea;
	// [Watchdog Timer Init Variable Declarations]$

	// $[WDTCN - Watchdog Timer Control]
	// Deprecated
	// [WDTCN - Watchdog Timer Control]$

	// $[WDTCN_2 - Watchdog Timer Control]

	// Feed WDT timer before disabling (Erratum WDT_E102)
	WDTCN = 0xA5;

	// Add 2 LFO cycle delay before disabling WDT (Erratum WDT_E102)
	for (i = 0; i < (2 * 3062500UL) / (10000 * 3); i++) {
		NOP();
	}

	// Disable WDT
	ea = IE_EA;
	IE_EA = 0;
	WDTCN = 0xDE;
	WDTCN = 0xAD;
	IE_EA = ea;

	// [WDTCN_2 - Watchdog Timer Control]$

}

//================================================================================
// PORTS_0_enter_DefaultMode_from_RESET
//================================================================================
extern void PORTS_0_enter_DefaultMode_from_RESET(void) {
	// $[P0 - Port 0 Pin Latch]
	// [P0 - Port 0 Pin Latch]$

	// $[P0MDOUT - Port 0 Output Mode]
	/***********************************************************************
	 - P0.0 output is push-pull
	 - P0.1 output is open-drain
	 - P0.2 output is open-drain
	 - P0.3 output is open-drain
	 - P0.4 output is push-pull
	 - P0.5 output is open-drain
	 - P0.6 output is open-drain
	 - P0.7 output is open-drain
	 ***********************************************************************/
	P0MDOUT = P0MDOUT_B0__PUSH_PULL | P0MDOUT_B1__OPEN_DRAIN
			| P0MDOUT_B2__OPEN_DRAIN | P0MDOUT_B3__OPEN_DRAIN
			| P0MDOUT_B4__PUSH_PULL | P0MDOUT_B5__OPEN_DRAIN
			| P0MDOUT_B6__OPEN_DRAIN | P0MDOUT_B7__OPEN_DRAIN;
	// [P0MDOUT - Port 0 Output Mode]$

	// $[P0MDIN - Port 0 Input Mode]
	/***********************************************************************
	 - P0.0 pin is configured for digital mode
	 - P0.1 pin is configured for digital mode
	 - P0.2 pin is configured for digital mode
	 - P0.3 pin is configured for digital mode
	 - P0.4 pin is configured for digital mode
	 - P0.5 pin is configured for digital mode
	 - P0.6 pin is configured for analog mode
	 - P0.7 pin is configured for analog mode
	 ***********************************************************************/
	P0MDIN = P0MDIN_B0__DIGITAL | P0MDIN_B1__DIGITAL | P0MDIN_B2__DIGITAL
			| P0MDIN_B3__DIGITAL | P0MDIN_B4__DIGITAL | P0MDIN_B5__DIGITAL
			| P0MDIN_B6__ANALOG | P0MDIN_B7__ANALOG;
	// [P0MDIN - Port 0 Input Mode]$

	// $[P0SKIP - Port 0 Skip]
	/***********************************************************************
	 - P0.0 pin is skipped by the crossbar
	 - P0.1 pin is skipped by the crossbar
	 - P0.2 pin is skipped by the crossbar
	 - P0.3 pin is skipped by the crossbar
	 - P0.4 pin is not skipped by the crossbar
	 - P0.5 pin is not skipped by the crossbar
	 - P0.6 pin is skipped by the crossbar
	 - P0.7 pin is skipped by the crossbar
	 ***********************************************************************/
	P0SKIP = P0SKIP_B0__SKIPPED | P0SKIP_B1__SKIPPED | P0SKIP_B2__SKIPPED
			| P0SKIP_B3__SKIPPED | P0SKIP_B4__NOT_SKIPPED
			| P0SKIP_B5__NOT_SKIPPED | P0SKIP_B6__SKIPPED | P0SKIP_B7__SKIPPED;
	// [P0SKIP - Port 0 Skip]$

	// $[P0MASK - Port 0 Mask]
	// [P0MASK - Port 0 Mask]$

	// $[P0MAT - Port 0 Match]
	// [P0MAT - Port 0 Match]$

}

//================================================================================
// PORTS_1_enter_DefaultMode_from_RESET
//================================================================================
extern void PORTS_1_enter_DefaultMode_from_RESET(void) {
	// $[P1 - Port 1 Pin Latch]
	// [P1 - Port 1 Pin Latch]$

	// $[P1MDOUT - Port 1 Output Mode]
	/***********************************************************************
	 - P1.0 output is open-drain
	 - P1.1 output is open-drain
	 - P1.2 output is push-pull
	 - P1.3 output is push-pull
	 - P1.4 output is push-pull
	 - P1.5 output is push-pull
	 - P1.6 output is push-pull
	 - P1.7 output is push-pull
	 ***********************************************************************/
	P1MDOUT = P1MDOUT_B0__OPEN_DRAIN | P1MDOUT_B1__OPEN_DRAIN
			| P1MDOUT_B2__PUSH_PULL | P1MDOUT_B3__PUSH_PULL
			| P1MDOUT_B4__PUSH_PULL | P1MDOUT_B5__PUSH_PULL
			| P1MDOUT_B6__PUSH_PULL | P1MDOUT_B7__PUSH_PULL;
	// [P1MDOUT - Port 1 Output Mode]$

	// $[P1MDIN - Port 1 Input Mode]
	/***********************************************************************
	 - P1.0 pin is configured for analog mode
	 - P1.1 pin is configured for digital mode
	 - P1.2 pin is configured for digital mode
	 - P1.3 pin is configured for digital mode
	 - P1.4 pin is configured for digital mode
	 - P1.5 pin is configured for digital mode
	 - P1.6 pin is configured for digital mode
	 - P1.7 pin is configured for digital mode
	 ***********************************************************************/
	P1MDIN = P1MDIN_B0__ANALOG | P1MDIN_B1__DIGITAL | P1MDIN_B2__DIGITAL
			| P1MDIN_B3__DIGITAL | P1MDIN_B4__DIGITAL | P1MDIN_B5__DIGITAL
			| P1MDIN_B6__DIGITAL | P1MDIN_B7__DIGITAL;
	// [P1MDIN - Port 1 Input Mode]$

	// $[P1SKIP - Port 1 Skip]
	/***********************************************************************
	 - P1.0 pin is skipped by the crossbar
	 - P1.1 pin is skipped by the crossbar
	 - P1.2 pin is not skipped by the crossbar
	 - P1.3 pin is not skipped by the crossbar
	 - P1.4 pin is skipped by the crossbar
	 - P1.5 pin is skipped by the crossbar
	 - P1.6 pin is skipped by the crossbar
	 - P1.7 pin is skipped by the crossbar
	 ***********************************************************************/
	P1SKIP = P1SKIP_B0__SKIPPED | P1SKIP_B1__SKIPPED | P1SKIP_B2__NOT_SKIPPED
			| P1SKIP_B3__NOT_SKIPPED | P1SKIP_B4__SKIPPED | P1SKIP_B5__SKIPPED
			| P1SKIP_B6__SKIPPED | P1SKIP_B7__SKIPPED;
	// [P1SKIP - Port 1 Skip]$

	// $[P1MASK - Port 1 Mask]
	/***********************************************************************
	 - P1.0 pin logic value is ignored and will not cause a port mismatch
	 event
	 - P1.1 pin logic value is compared to P1MAT.1
	 - P1.2 pin logic value is ignored and will not cause a port mismatch
	 event
	 - P1.3 pin logic value is ignored and will not cause a port mismatch
	 event
	 - P1.4 pin logic value is ignored and will not cause a port mismatch
	 event
	 - P1.5 pin logic value is ignored and will not cause a port mismatch
	 event
	 - P1.6 pin logic value is ignored and will not cause a port mismatch
	 event
	 - P1.7 pin logic value is ignored and will not cause a port mismatch
	 event
	 ***********************************************************************/
	P1MASK = P1MASK_B0__IGNORED | P1MASK_B1__COMPARED | P1MASK_B2__IGNORED
			| P1MASK_B3__IGNORED | P1MASK_B4__IGNORED | P1MASK_B5__IGNORED
			| P1MASK_B6__IGNORED | P1MASK_B7__IGNORED;
	// [P1MASK - Port 1 Mask]$

	// $[P1MAT - Port 1 Match]
	// [P1MAT - Port 1 Match]$

}

//================================================================================
// PBCFG_0_enter_DefaultMode_from_RESET
//================================================================================
extern void PBCFG_0_enter_DefaultMode_from_RESET(void) {
	// $[XBR2 - Port I/O Crossbar 2]
	/***********************************************************************
	 - Weak Pullups enabled 
	 - Crossbar enabled
	 ***********************************************************************/
	XBR2 = XBR2_WEAKPUD__PULL_UPS_ENABLED | XBR2_XBARE__ENABLED;
	// [XBR2 - Port I/O Crossbar 2]$

	// $[PRTDRV - Port Drive Strength]
	// [PRTDRV - Port Drive Strength]$

	// $[XBR0 - Port I/O Crossbar 0]
	/***********************************************************************
	 - UART TX, RX routed to Port pins P0.4 and P0.5
	 - SPI I/O unavailable at Port pins
	 - SMBus 0 I/O unavailable at Port pins
	 - CP0 unavailable at Port pin
	 - Asynchronous CP0 unavailable at Port pin
	 - CP1 unavailable at Port pin
	 - Asynchronous CP1 unavailable at Port pin
	 - SYSCLK unavailable at Port pin
	 ***********************************************************************/
	XBR0 = XBR0_URT0E__ENABLED | XBR0_SPI0E__DISABLED | XBR0_SMB0E__DISABLED
			| XBR0_CP0E__DISABLED | XBR0_CP0AE__DISABLED | XBR0_CP1E__DISABLED
			| XBR0_CP1AE__DISABLED | XBR0_SYSCKE__DISABLED;
	// [XBR0 - Port I/O Crossbar 0]$

	// $[XBR1 - Port I/O Crossbar 1]
	/***********************************************************************
	 - CEX0, CEX1 routed to Port pins
	 - ECI unavailable at Port pin
	 - T0 unavailable at Port pin
	 - T1 unavailable at Port pin
	 - T2 unavailable at Port pin
	 ***********************************************************************/
	XBR1 = XBR1_PCA0ME__CEX0_CEX1 | XBR1_ECIE__DISABLED | XBR1_T0E__DISABLED
			| XBR1_T1E__DISABLED | XBR1_T2E__DISABLED;
	// [XBR1 - Port I/O Crossbar 1]$

}

//================================================================================
// CLOCK_0_enter_DefaultMode_from_RESET
//================================================================================
extern void CLOCK_0_enter_DefaultMode_from_RESET(void) {
	// $[CLKSEL - Clock Select]
	/***********************************************************************
	 - Clock derived from the Internal High-Frequency Oscillator
	 - SYSCLK is equal to selected clock source divided by 1
	 ***********************************************************************/
	CLKSEL = CLKSEL_CLKSL__HFOSC | CLKSEL_CLKDIV__SYSCLK_DIV_1;
	// [CLKSEL - Clock Select]$

}

//================================================================================
// PCA_0_enter_DefaultMode_from_RESET
//================================================================================
extern void PCA_0_enter_DefaultMode_from_RESET(void) {
	// $[PCA Off]
	PCA0CN0_CR = PCA0CN0_CR__STOP;
	// [PCA Off]$

	// $[PCA0MD - PCA Mode]
	/***********************************************************************
	 - PCA continues to function normally while the system controller is in
	 Idle Mode
	 - Enable a PCA Counter/Timer Overflow interrupt request when CF is set
	 - System clock
	 ***********************************************************************/
	PCA0MD = PCA0MD_CIDL__NORMAL | PCA0MD_ECF__OVF_INT_ENABLED
			| PCA0MD_CPS__SYSCLK;
	// [PCA0MD - PCA Mode]$

	// $[PCA0CENT - PCA Center Alignment Enable]
	/***********************************************************************
	 - Center-aligned
	 - Edge-aligned
	 - Edge-aligned
	 ***********************************************************************/
	PCA0CENT = PCA0CENT_CEX0CEN__CENTER | PCA0CENT_CEX1CEN__EDGE
			| PCA0CENT_CEX2CEN__EDGE;
	// [PCA0CENT - PCA Center Alignment Enable]$

	// $[PCA0CLR - PCA Comparator Clear Control]
	/***********************************************************************
	 - PCA channel
	 - Enable the comparator clear function on PCA channel 0
	 - Disable the comparator clear function on PCA channel 1
	 - Disable the comparator clear function on PCA channel 2
	 ***********************************************************************/
	PCA0CLR = PCA0CLR_CPCPOL__LOW | PCA0CLR_CPCE0__ENABLED
			| PCA0CLR_CPCE1__DISABLED | PCA0CLR_CPCE2__DISABLED;
	// [PCA0CLR - PCA Comparator Clear Control]$

	// $[PCA0L - PCA Counter/Timer Low Byte]
	// [PCA0L - PCA Counter/Timer Low Byte]$

	// $[PCA0H - PCA Counter/Timer High Byte]
	/***********************************************************************
	 - PCA Counter/Timer High Byte = 0x04
	 ***********************************************************************/
	PCA0H = (0x04 << PCA0H_PCA0H__SHIFT);
	// [PCA0H - PCA Counter/Timer High Byte]$

	// $[PCA0POL - PCA Output Polarity]
	// [PCA0POL - PCA Output Polarity]$

	// $[PCA0PWM - PCA PWM Configuration]
	// [PCA0PWM - PCA PWM Configuration]$

	// $[PCA On]
	PCA0CN0_CR = PCA0CN0_CR__RUN;
	// [PCA On]$

}

//================================================================================
// PCACH_0_enter_DefaultMode_from_RESET
//================================================================================
extern void PCACH_0_enter_DefaultMode_from_RESET(void) {
	// $[PCA0 Settings Save]
	// Select Capture/Compare register)
	PCA0PWM &= ~PCA0PWM_ARSEL__BMASK;
	// [PCA0 Settings Save]$

	// $[PCA0CPM0 - PCA Channel 0 Capture/Compare Mode]
	/***********************************************************************
	 - Disable negative edge capture
	 - Enable a Capture/Compare Flag interrupt request when CCF0 is set
	 - Enable match function
	 - 8 to 11-bit PWM selected
	 - Disable positive edge capture
	 - Disable comparator function
	 - Enable PWM function
	 - Disable toggle function
	 ***********************************************************************/
	PCA0CPM0 = PCA0CPM0_CAPN__DISABLED | PCA0CPM0_ECCF__ENABLED
			| PCA0CPM0_MAT__ENABLED | PCA0CPM0_PWM16__8_BIT
			| PCA0CPM0_CAPP__DISABLED | PCA0CPM0_ECOM__DISABLED
			| PCA0CPM0_PWM__ENABLED | PCA0CPM0_TOG__DISABLED;
	// [PCA0CPM0 - PCA Channel 0 Capture/Compare Mode]$

	// $[PCA0CPL0 - PCA Channel 0 Capture Module Low Byte]
	// [PCA0CPL0 - PCA Channel 0 Capture Module Low Byte]$

	// $[PCA0CPH0 - PCA Channel 0 Capture Module High Byte]
	PCA0CPH0 = 0x00;
	// [PCA0CPH0 - PCA Channel 0 Capture Module High Byte]$

	// $[Auto-reload]
	// [Auto-reload]$

	// $[PCA0 Settings Restore]
	// [PCA0 Settings Restore]$

}

extern void ADC_0_enter_DefaultMode_from_RESET(void) {
	// $[ADC0CN1 - ADC0 Control 1]
	// [ADC0CN1 - ADC0 Control 1]$

	// $[ADC0MX - ADC0 Multiplexer Selection]
	/***********************************************************************
	 - Select ADC0.8
	 ***********************************************************************/
	ADC0MX = ADC0MX_ADC0MX__ADC0P8;
	// [ADC0MX - ADC0 Multiplexer Selection]$

	// $[ADC0CF - ADC0 Configuration]
	// [ADC0CF - ADC0 Configuration]$

	// $[ADC0AC - ADC0 Accumulator Configuration]
	// [ADC0AC - ADC0 Accumulator Configuration]$

	// $[ADC0TK - ADC0 Burst Mode Track Time]
	// [ADC0TK - ADC0 Burst Mode Track Time]$

	// $[ADC0PWR - ADC0 Power Control]
	// [ADC0PWR - ADC0 Power Control]$

	// $[ADC0GTH - ADC0 Greater-Than High Byte]
	// [ADC0GTH - ADC0 Greater-Than High Byte]$

	// $[ADC0GTL - ADC0 Greater-Than Low Byte]
	// [ADC0GTL - ADC0 Greater-Than Low Byte]$

	// $[ADC0LTH - ADC0 Less-Than High Byte]
	// [ADC0LTH - ADC0 Less-Than High Byte]$

	// $[ADC0LTL - ADC0 Less-Than Low Byte]
	// [ADC0LTL - ADC0 Less-Than Low Byte]$

	// $[ADC0CN0 - ADC0 Control 0]
	/***********************************************************************
	 - Enable ADC0 
	 - Enable ADC0 burst mode
	 - ADC0 conversion initiated on overflow of Timer 2
	 ***********************************************************************/
	ADC0CN0 &= ~ADC0CN0_ADCM__FMASK;
	ADC0CN0 |= ADC0CN0_ADEN__ENABLED | ADC0CN0_ADBMEN__BURST_ENABLED
			| ADC0CN0_ADCM__TIMER2;
	// [ADC0CN0 - ADC0 Control 0]$

}

extern void CMP_1_enter_DefaultMode_from_RESET(void) {
	// $[CMP1MX - Comparator 1 Multiplexer Selection]
	// [CMP1MX - Comparator 1 Multiplexer Selection]$

	// $[CMP1MD - Comparator 1 Mode]
	/***********************************************************************
	 - Mode 0 
	 - Comparator falling-edge interrupt enabled
	 - Comparator rising-edge interrupt enabled
	 ***********************************************************************/
	CMP1MD &= ~CMP1MD_CPMD__FMASK;
	CMP1MD |= CMP1MD_CPFIE__FALL_INT_ENABLED | CMP1MD_CPRIE__RISE_INT_ENABLED;
	// [CMP1MD - Comparator 1 Mode]$

	// $[CMP1CN0 - Comparator 1 Control 0]
	/***********************************************************************
	 - Comparator enabled
	 ***********************************************************************/
	CMP1CN0 |= CMP1CN0_CPEN__ENABLED;
	// [CMP1CN0 - Comparator 1 Control 0]$

}

extern void TIMER01_0_enter_DefaultMode_from_RESET(void) {
	// $[Timer Initialization]
	//Save Timer Configuration
	uint8_t TCON_save;
	TCON_save = TCON;
	//Stop Timers
	TCON &= ~TCON_TR0__BMASK & ~TCON_TR1__BMASK;

	// [Timer Initialization]$

	// $[TH0 - Timer 0 High Byte]
	// [TH0 - Timer 0 High Byte]$

	// $[TL0 - Timer 0 Low Byte]
	// [TL0 - Timer 0 Low Byte]$

	// $[TH1 - Timer 1 High Byte]
	/***********************************************************************
	 - Timer 1 High Byte = 0x96
	 ***********************************************************************/
	TH1 = (0x96 << TH1_TH1__SHIFT);
	// [TH1 - Timer 1 High Byte]$

	// $[TL1 - Timer 1 Low Byte]
	// [TL1 - Timer 1 Low Byte]$

	// $[Timer Restoration]
	//Restore Timer Configuration
	TCON |= (TCON_save & TCON_TR0__BMASK) | (TCON_save & TCON_TR1__BMASK);

	// [Timer Restoration]$

}

extern void TIMER16_2_enter_DefaultMode_from_RESET(void) {
	// $[Timer Initialization]
	// Save Timer Configuration
	uint8_t TMR2CN0_TR2_save;
	TMR2CN0_TR2_save = TMR2CN0 & TMR2CN0_TR2__BMASK;
	// Stop Timer
	TMR2CN0 &= ~(TMR2CN0_TR2__BMASK);
	// [Timer Initialization]$

	// $[TMR2CN0 - Timer 2 Control]
	// [TMR2CN0 - Timer 2 Control]$

	// $[TMR2H - Timer 2 High Byte]
	// [TMR2H - Timer 2 High Byte]$

	// $[TMR2L - Timer 2 Low Byte]
	// [TMR2L - Timer 2 Low Byte]$

	// $[TMR2RLH - Timer 2 Reload High Byte]
	/***********************************************************************
	 - Timer 2 Reload High Byte = 0xFF
	 ***********************************************************************/
	TMR2RLH = (0xFF << TMR2RLH_TMR2RLH__SHIFT);
	// [TMR2RLH - Timer 2 Reload High Byte]$

	// $[TMR2RLL - Timer 2 Reload Low Byte]
	/***********************************************************************
	 - Timer 2 Reload Low Byte = 0x34
	 ***********************************************************************/
	TMR2RLL = (0x34 << TMR2RLL_TMR2RLL__SHIFT);
	// [TMR2RLL - Timer 2 Reload Low Byte]$

	// $[TMR2CN0]
	/***********************************************************************
	 - Start Timer 2 running
	 ***********************************************************************/
	TMR2CN0 |= TMR2CN0_TR2__RUN;
	// [TMR2CN0]$

	// $[Timer Restoration]
	// Restore Timer Configuration
	TMR2CN0 |= TMR2CN0_TR2_save;
	// [Timer Restoration]$

}

extern void TIMER_SETUP_0_enter_DefaultMode_from_RESET(void) {
	// $[CKCON0 - Clock Control 0]
	/***********************************************************************
	 - System clock divided by 12
	 - Counter/Timer 0 uses the clock defined by the prescale field, SCA
	 - Timer 2 high byte uses the clock defined by T2XCLK in TMR2CN0
	 - Timer 2 low byte uses the clock defined by T2XCLK in TMR2CN0
	 - Timer 3 high byte uses the clock defined by T3XCLK in TMR3CN0
	 - Timer 3 low byte uses the clock defined by T3XCLK in TMR3CN0
	 - Timer 1 uses the system clock
	 ***********************************************************************/
	CKCON0 = CKCON0_SCA__SYSCLK_DIV_12 | CKCON0_T0M__PRESCALE
			| CKCON0_T2MH__EXTERNAL_CLOCK | CKCON0_T2ML__EXTERNAL_CLOCK
			| CKCON0_T3MH__EXTERNAL_CLOCK | CKCON0_T3ML__EXTERNAL_CLOCK
			| CKCON0_T1M__SYSCLK;
	// [CKCON0 - Clock Control 0]$

	// $[TMOD - Timer 0/1 Mode]
	/***********************************************************************
	 - Mode 0, 13-bit Counter/Timer
	 - Mode 2, 8-bit Counter/Timer with Auto-Reload
	 - Timer Mode
	 - Timer 0 enabled when TR0 = 1 irrespective of INT0 logic level
	 - Timer Mode
	 - Timer 1 enabled when TR1 = 1 irrespective of INT1 logic level
	 ***********************************************************************/
	TMOD = TMOD_T0M__MODE0 | TMOD_T1M__MODE2 | TMOD_CT0__TIMER
			| TMOD_GATE0__DISABLED | TMOD_CT1__TIMER | TMOD_GATE1__DISABLED;
	// [TMOD - Timer 0/1 Mode]$

	// $[TCON - Timer 0/1 Control]
	/***********************************************************************
	 - Start Timer 1 running
	 ***********************************************************************/
	TCON |= TCON_TR1__RUN;
	// [TCON - Timer 0/1 Control]$

}

extern void UART_0_enter_DefaultMode_from_RESET(void) {
	// $[SCON0 - UART0 Serial Port Control]
	/***********************************************************************
	 - UART0 reception enabled
	 ***********************************************************************/
	SCON0 |= SCON0_REN__RECEIVE_ENABLED;
	// [SCON0 - UART0 Serial Port Control]$

}

extern void INTERRUPT_0_enter_DefaultMode_from_RESET(void) {
	// $[EIE1 - Extended Interrupt Enable 1]
	/***********************************************************************
	 - Enable interrupt requests generated by the ADINT flag
	 - Disable ADC0 Window Comparison interrupt
	 - Enable interrupt requests generated by the comparator 0 CPRIF or CPFIF
	 flags
	 - Disable CP1 interrupts
	 - Disable all Port Match interrupts
	 - Enable interrupt requests generated by PCA0
	 - Disable all SMB0 interrupts
	 - Enable interrupt requests generated by the TF3L or TF3H flags
	 ***********************************************************************/
	EIE1 = EIE1_EADC0__ENABLED | EIE1_EWADC0__DISABLED | EIE1_ECP0__ENABLED
			| EIE1_ECP1__DISABLED | EIE1_EMAT__DISABLED | EIE1_EPCA0__ENABLED
			| EIE1_ESMB0__DISABLED | EIE1_ET3__ENABLED;
	// [EIE1 - Extended Interrupt Enable 1]$

	// $[EIP1 - Extended Interrupt Priority 1]
	// [EIP1 - Extended Interrupt Priority 1]$

	// $[IE - Interrupt Enable]
	/***********************************************************************
	 - Enable each interrupt according to its individual mask setting
	 - Disable external interrupt 0
	 - Disable external interrupt 1
	 - Disable all SPI0 interrupts
	 - Enable interrupt requests generated by the TF0 flag
	 - Enable interrupt requests generated by the TF1 flag
	 - Disable Timer 2 interrupt
	 - Enable UART0 interrupt
	 ***********************************************************************/
	IE = IE_EA__ENABLED | IE_EX0__DISABLED | IE_EX1__DISABLED
			| IE_ESPI0__DISABLED | IE_ET0__ENABLED | IE_ET1__ENABLED
			| IE_ET2__DISABLED | IE_ES0__ENABLED;
	// [IE - Interrupt Enable]$

	// $[IP - Interrupt Priority]
	// [IP - Interrupt Priority]$

}

extern void TIMER16_3_enter_DefaultMode_from_RESET(void) {
	// $[Timer Initialization]
	// Save Timer Configuration
	uint8_t TMR3CN0_TR3_save;
	TMR3CN0_TR3_save = TMR3CN0 & TMR3CN0_TR3__BMASK;
	// Stop Timer
	TMR3CN0 &= ~(TMR3CN0_TR3__BMASK);
	// [Timer Initialization]$

	// $[TMR3CN0 - Timer 3 Control]
	// [TMR3CN0 - Timer 3 Control]$

	// $[TMR3H - Timer 3 High Byte]
	// [TMR3H - Timer 3 High Byte]$

	// $[TMR3L - Timer 3 Low Byte]
	// [TMR3L - Timer 3 Low Byte]$

	// $[TMR3RLH - Timer 3 Reload High Byte]
	// [TMR3RLH - Timer 3 Reload High Byte]$

	// $[TMR3RLL - Timer 3 Reload Low Byte]
	// [TMR3RLL - Timer 3 Reload Low Byte]$

	// $[TMR3CN0]
	/***********************************************************************
	 - Start Timer 3 running
	 ***********************************************************************/
	TMR3CN0 |= TMR3CN0_TR3__RUN;
	// [TMR3CN0]$

	// $[Timer Restoration]
	// Restore Timer Configuration
	TMR3CN0 |= TMR3CN0_TR3_save;
	// [Timer Restoration]$

}

extern void PORTS_2_enter_DefaultMode_from_RESET(void) {

	// $[P2 - Port 2 Pin Latch]
	// [P2 - Port 2 Pin Latch]$

	// $[P2MDOUT - Port 2 Output Mode]
	// [P2MDOUT - Port 2 Output Mode]$

}

extern void CMP_0_enter_DefaultMode_from_RESET(void) {
	// $[CMP0MX - Comparator 0 Multiplexer Selection]
	// [CMP0MX - Comparator 0 Multiplexer Selection]$

	// $[CMP0MD - Comparator 0 Mode]
	/***********************************************************************
	 - Mode 0 
	 - Comparator falling-edge interrupt enabled
	 - Comparator rising-edge interrupt enabled
	 ***********************************************************************/
	CMP0MD &= ~CMP0MD_CPMD__FMASK;
	CMP0MD |= CMP0MD_CPFIE__FALL_INT_ENABLED | CMP0MD_CPRIE__RISE_INT_ENABLED;
	// [CMP0MD - Comparator 0 Mode]$

	// $[CMP0CN0 - Comparator 0 Control 0]
	/***********************************************************************
	 - Comparator enabled
	 ***********************************************************************/
	CMP0CN0 |= CMP0CN0_CPEN__ENABLED;
	// [CMP0CN0 - Comparator 0 Control 0]$

}


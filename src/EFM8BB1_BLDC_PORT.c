//-----------------------------------------------------------------------------
// PCA0_8Bit_PWM_Output.c
//-----------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include "bsp.h"
#include "InitDevice.h"
#include "pca_0.h"

//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
  // Disable the watchdog here
}
 
//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------

void main(void)
{
  uint16_t delay_count;                       // Used to implement a delay
  bool duty_direction = 0;                     // 0 = Decrease; 1 = Increase
  uint8_t duty_cycle = 0x80;

  enter_DefaultMode_from_RESET();

  while (1)
  {
    // Wait a little while
    for (delay_count = 15000; delay_count > 0; delay_count--);

    if (duty_direction == 1)                  // Direction = Increase
    {
      // First, check the ECOM0 bit
      if ((PCA0CPM0 & PCA0CPM0_ECOM__BMASK) == PCA0CPM0_ECOM__DISABLED)
      {
        PCA0CPM0 |= PCA0CPM0_ECOM__ENABLED;   // Set ECOM0 if it is '0'
      }
      else                                    // Increase duty cycle otherwise
      {
        duty_cycle--;                         // Increase duty cycle

        PCA0_writeChannel(PCA0_CHAN0, duty_cycle << 8);


        if (PCA0CPH0 == 0x00)
        {
          duty_direction = 0;                 // Change direction for next time
        }
      }
    }
    else                                      // Direction = Decrease
    {
      if (duty_cycle == 0xFF)
      {
        PCA0CPM0 &= ~PCA0CPM0_ECOM__BMASK;    // Clear ECOM0
        duty_direction = 1;                   // Change direction for next time
      }
      else
      {
        duty_cycle++;                         // Decrease duty cycle

        PCA0_writeChannel(PCA0_CHAN0, duty_cycle << 8);
      }
    }

  }
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

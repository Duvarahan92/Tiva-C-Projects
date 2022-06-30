#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "tm4c123GH6PM_gpio_driver.h"

//*****************************************************************************
//
// Blink the on-board LED with interrupt.
//
//*****************************************************************************

int main(void)
{
  // Enable clock for port F
  GPIO_EnableClk(SYSCTL_RCGCGPIO_R5);

  // LED Config
  GPIO_ModeSet(GPIOF_P, GPIO_PIN_1, GPIO_OUT);
  GPIO_PadConfig(GPIOF_P, GPIO_PIN_1, GPIO_NO_DR, GPIO_DEN);
 
  // Button Config
  //unlocks pin 0 on port F
  GPIO_Unlock(GPIOF_P, GPIO_PIN_0);
  
  GPIO_ModeSet(GPIOF_P, GPIO_PIN_0, GPIO_IN);
  GPIO_PadConfig(GPIOF_P, GPIO_PIN_0, GPIO_NO_DR, GPIO_PUR);

  // IRQ configurations
  GPIO_IRQPriorityConfig(INT_GPIOF, 3);
  GPIO_IRQConfig(INT_GPIOF, ENABLE);

  // Initialize interrupt
  GPIO_DisableInterrupt(GPIOF_P, GPIO_PIN_0);
  GPIO_InterruptTypeSet(GPIOF_P, GPIO_PIN_0, GPIO_FALLING_EDGE);
  GPIO_ClearInterrupt(GPIOF_P, GPIO_PIN_0);
  GPIO_EnableInterrupt(GPIOF_P, GPIO_PIN_0);
  
  while (1)
  {
    
  }
  
  return 0;
}

   void GPIOPortF_IRQHandler(void) 
  {
    GPIO_IRQHandling(GPIOF_P, GPIO_PIN_0);
    GPIO_ToggleOutputPin(GPIOF_P, GPIO_PIN_1);
  }
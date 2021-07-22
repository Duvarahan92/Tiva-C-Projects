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

  //GPIO LED and GPIO Button
  GPIO_Handle_t GPIOLed, GPIOButton;

  memset(&GPIOLed, 0, sizeof(GPIOLed));
  memset(&GPIOButton, 0, sizeof(GPIOButton));
  
  GPIOLed.GPIOx = GPIOF;
  GPIOEnableClk(SYSCTL_RCGCGPIO_R5);

  GPIOModeSet(GPIOF_P, GPIO_PIN_1, GPIO_OUT);
  GPIOPadConfig(GPIOF_P, GPIO_PIN_1, GPIO_NO_DR, GPIO_DEN);
 
  GPIOButton.GPIOx = GPIOF;
  
  //unlocks pf0
  GPIO_Unlock(GPIOF_P, GPIO_PIN_0);
  
  GPIOModeSet(GPIOF_P, GPIO_PIN_0, GPIO_IN);
  GPIOPadConfig(GPIOF_P, GPIO_PIN_0, GPIO_NO_DR, GPIO_PUR);

  // IRQ configurations
  GPIO_IRQPriorityConfig(INT_GPIOF, 3);
  GPIO_IRQConfig(INT_GPIOF, ENABLE);

  GPIO_InterruptInit(&GPIOButton);
  while (1)
  {
    
  }
  
  return 0;
}

   void GPIOPortF_IRQHandler(void) 
  {
    GPIO_IRQHandling(GPIOF, 0);
    GPIO_ToggleOutputPin(GPIOF_P, GPIO_PIN_1);
  }
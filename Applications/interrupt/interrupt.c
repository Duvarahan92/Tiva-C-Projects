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
  GPIO_CLK_CTRL(GPIOF, ENABLE);
 
  GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 1;
  GPIOLed.GPIO_PinConfig.GPIO_PinDir = GPIO_OUT;
  GPIOLed.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;
 

  GPIOButton.GPIOx = GPIOF;
  GPIOButton.GPIO_PinConfig.GPIO_PinNumber = 0;
  
  //unlocks pf0
  GPIO_Unlock(GPIOButton.GPIOx, GPIOButton.GPIO_PinConfig.GPIO_PinNumber);
  
  GPIOButton.GPIO_PinConfig.GPIO_PinDir = GPIO_IN;
  GPIOButton.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;
  GPIOButton.GPIO_PinConfig.GPIO_PUR = GPIO_PUR_SET;

  // IRQ configurations
  GPIO_IRQPriorityConfig(INT_GPIOF, 3);
  GPIO_IRQConfig(INT_GPIOF, ENABLE);

  GPIO_Init(&GPIOLed);
  GPIO_Init(&GPIOButton);
  GPIO_InterruptInit(&GPIOButton);

  while (1)
  {
  
  }
  
  return 0;
}

   void GPIOPortF_IRQHandler(void) 
  {
    GPIO_IRQHandling(GPIOF, 0);
    GPIO_ToggleOutputPin(GPIOF, 1);
  }
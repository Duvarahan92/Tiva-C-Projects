#include <stdint.h>
#include <stdio.h>
#include "tm4c123GH6PM_gpio_driver.h"

//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************

void delay(uint32_t ticks) 
{
  for (size_t i = 0; i < ticks; i++)
  {
    
  }
}

int main(void)
{
  //GPIO LED
  GPIO_Handle_t GPIOLed;
  
  GPIOLed.GPIOx = GPIOF;
  GPIOEnableClk(SYSCTL_RCGCGPIO_R5);
 
  GPIOModeSet(GPIOF_P, GPIO_PIN_3, GPIO_OUT);
  GPIOPadConfig(GPIOF_P, GPIO_PIN_3, GPIO_NO_DR, GPIO_DEN);
  /*GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 1;
  GPIOLed.GPIO_PinConfig.GPIO_PinDir = GPIO_OUT;
  GPIOLed.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;*/

  GPIO_Init(&GPIOLed);

  while (1)
  {
    delay(500000/2);
    GPIO_ToggleOutputPin(GPIOF_P, GPIO_PIN_3);
  }
  
  return 0;
}

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
  GPIO_CLK_CTRL(GPIOF, ENABLE);
 
  GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 1;
  GPIOLed.GPIO_PinConfig.GPIO_PinDir = GPIO_OUT;
  GPIOLed.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;

  GPIO_Init(&GPIOLed);

  while (1)
  {
    delay(500000/2);
    GPIO_ToggleOutputPin(GPIOLed.GPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
  }
  
  return 0;
}

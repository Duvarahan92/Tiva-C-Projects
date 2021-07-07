#include <stdint.h>
#include <stdio.h>
#include "tm4c123GH6PM_gpio_driver.h"

//*****************************************************************************
//
//
//*****************************************************************************

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
  GPIO_Handle_t GPIOLed;

  GPIOLed.GPIOx = GPIOF;
  GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 2;
  GPIOLed.GPIO_PinConfig.GPIO_PinDir = GPIO_OUT;
  GPIOLed.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;

  GPIO_CLK_CTRL(GPIOLed.GPIOx, ENABLE);

  GPIO_Init(&GPIOLed);

  while (1)
  {
    GPIO_ToggleOutputPin(GPIOLed.GPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
    delay(500000);
  }
  

  return 0;
}

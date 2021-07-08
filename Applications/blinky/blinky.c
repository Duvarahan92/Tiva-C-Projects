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
  GPIO_Handle_t GPIOLed;
  
  GPIOLed.GPIOx = GPIOF;
  GPIO_DeInit(GPIOF);
  GPIO_CLK_CTRL(GPIOF, ENABLE);
 
  GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 2;
  GPIOLed.GPIO_PinConfig.GPIO_PinDir = GPIO_OUT;
  GPIOLed.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;
  GPIOLed.GPIO_PinConfig.GPIO_OODR = 0;
  GPIOLed.GPIO_PinConfig.GPIO_PUR = 0;
  GPIOLed.GPIO_PinConfig.GPIO_PDR = 0;
  GPIOLed.GPIO_PinConfig.GPIO_AFSEL = 0;
  
  GPIO_Init(&GPIOLed);

  while (1)
  {
    GPIO_ToggleOutputPin(GPIOLed.GPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
    delay(500000);
  }
  

  return 0;
}

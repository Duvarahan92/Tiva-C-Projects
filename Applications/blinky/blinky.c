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
  // Switch button
  uint32_t SW1;

  //GPIO LED
  GPIO_Handle_t GPIOLed;
  
  GPIOLed.GPIOx = GPIOF;
  GPIO_CLK_CTRL(GPIOF, ENABLE);
 
  GPIOLed.GPIO_PinConfig.GPIO_PinNumber = 1;
  GPIOLed.GPIO_PinConfig.GPIO_PinDir = GPIO_OUT;
  GPIOLed.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;
  GPIOLed.GPIO_PinConfig.GPIO_OODR = 0;
  GPIOLed.GPIO_PinConfig.GPIO_PUR = 0;
  GPIOLed.GPIO_PinConfig.GPIO_PDR = 0;
  GPIOLed.GPIO_PinConfig.GPIO_AFSEL = 0;

  //GPIO Button
  GPIO_Handle_t GPIOButton;
  GPIOButton.GPIOx = GPIOF;
  GPIOButton.GPIO_PinConfig.GPIO_PinNumber = 0;
  
  //unlocks pf0
  GPIO_UNlock(GPIOButton.GPIOx, GPIOButton.GPIO_PinConfig.GPIO_PinNumber);
  
  GPIOButton.GPIO_PinConfig.GPIO_AFSEL = 0;
  GPIOButton.GPIO_PinConfig.GPIO_AMSEL = GPIO_AMSEL_CLEAR;
  GPIOButton.GPIO_PinConfig.GPIO_PinDir = 0;
  GPIOButton.GPIO_PinConfig.GPIO_DEN = GPIO_DEN_SET;
  GPIOButton.GPIO_PinConfig.GPIO_OODR = 0;
  GPIOButton.GPIO_PinConfig.GPIO_PUR = 1;
  GPIOButton.GPIO_PinConfig.GPIO_PDR = 0;
  
  

  GPIO_Init(&GPIOLed);
  GPIO_Init(&GPIOButton);

  while (1)
  {
    SW1 = GPIO_ReadInputPin(GPIOButton.GPIOx, GPIOButton.GPIO_PinConfig.GPIO_PinNumber);
    if (!SW1) {
      delay(500000/2);
      GPIO_ToggleOutputPin(GPIOLed.GPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
    }
  }
  

  return 0;
}

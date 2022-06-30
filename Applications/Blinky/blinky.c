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
  for (size_t i = 0; i < ticks; i++);
}

int main(void)
{
  GPIO_EnableClk(SYSCTL_RCGCGPIO_R5);
 
  GPIO_ModeSet(GPIOF_P, GPIO_PIN_3, GPIO_OUT);
  GPIO_PadConfig(GPIOF_P, GPIO_PIN_3, GPIO_NO_DR, GPIO_DEN);

  while (1)
  {
    delay(500000/2);
    GPIO_ToggleOutputPin(GPIOF_P, GPIO_PIN_3);
  }
  
  return 0;
}

#include <stdint.h>
#include <stdio.h>
#include "tm4c123GH6PM_gpio_driver.h"
#include "tm4c123GH6PM_uart_driver.h"

int main(void)
{
    uint32_t SysClk = 16000000;
    uint32_t BaudRate = 115200;
    unsigned char word[] = "Hello world";

    // Activate clock for UART module 0 and GPIO port A and F
    UART_EnableClk(SYSCTL_RCGCUART_R0);
    GPIO_EnableClk(SYSCTL_RCGCGPIO_R0);
    GPIO_EnableClk(SYSCTL_RCGCGPIO_R5);

    //Config UART 0
    // 80 MHz clock, 115200 baud rate, no parity, 8 bit word length and
    // one stop bit
    UART_ConfigModule(UART0_P, SysClk, BaudRate, UART_LCRH_NONE, UART_LCRH_WLEN_8, UART_LCRH_STP1);

    //Configure pin 0 and 1 on GPIO port A as UART
    GPIO_UARTType(GPIOA_P, GPIO_PIN_0, GPIO_PCTL_PA0_U0RX);
    GPIO_UARTType(GPIOA_P, GPIO_PIN_1, GPIO_PCTL_PA1_U0TX);
    
    // Button Config
    //unlocks pin 0 on port F
    GPIO_Unlock(GPIOF_P, GPIO_PIN_0);
  
    GPIO_ModeSet(GPIOF_P, GPIO_PIN_0, GPIO_IN);
    GPIO_PadConfig(GPIOF_P, GPIO_PIN_0, GPIO_NO_DR, GPIO_PUR);

    // Write to PC
    UART_WriteString(UART0_P, word);

    while (1)
     {
         int32_t read = UART_ReadChar(UART0_P);
         UART_WriteChar(UART0_P, read);
     }

    return 0;
}
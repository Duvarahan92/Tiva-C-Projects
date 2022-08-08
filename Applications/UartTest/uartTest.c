#include <stdint.h>
#include <stdio.h>
#include "tm4c123GH6PM_uart_driver.h"

#define SysClk              16000000
#define BaudRate            115200     

int main(void)
{
    static char word[] = "Hello world\r\n";

    //Config UART 0
    // 16 MHz clock, 115200 baud rate, no parity, 8 bit word length and
    // one stop bit
    UART_Init(UART0_P, SysClk, BaudRate, UART_LCRH_NONE, UART_LCRH_WLEN_8, UART_LCRH_STP1);

    // Write to PC
    UART_WriteString(UART0_P, word);

    while (1)
     {
         char read = UART_ReadChar(UART0_P);
         UART_WriteChar(UART0_P, read);
     }

    return 0;
}
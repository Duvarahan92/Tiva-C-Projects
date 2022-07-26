#include <stdint.h>
#include <stdio.h>
#include "tm4c123GH6PM_spi_driver.h"
#include "tm4c123GH6PM_uart_driver.h"

void delay(uint32_t ticks) 
{
  for (size_t i = 0; i < ticks; i++);
}

int main(void)
{
    static const uint32_t SysClk = 16000000;
    static const uint32_t BaudRate = 115200;
    static const uint32_t BitRate = 1000000;
    static const unsigned char word[] = "The definition of insanity is doing the same thing over and over again and expecting different results.";
    static uint8_t store;

    //Config and init UART 0
    // 16 MHz clock, 115200 baud rate, no parity, 8 bit word length and
    // one stop bit
    UART_Init(UART0_P, SysClk, BaudRate, UART_LCRH_NONE, UART_LCRH_WLEN_8, UART_LCRH_STP1);

    // Init SPI 0 select master mode
    SSI_Init(SSI0_P, SSI_CR1_M);

    // Configure SSI_0 module
    SSI_ConfigModule(SSI0_P, SSI_CR0_MODE_0, SSI_CR0_FRF_MOTO, SSI_CR0_DSS_8, SysClk, BitRate);

    // Enable loopback
    EnableLoopbackMode(SSI0_P);

    // Write-read SPI
    for (int i = 0; i < sizeof(word); i++)
    {
        SSI_SendData(SSI0_P, word[i]);
        SSI_ReceiveData(SSI0_P, &store);
        UART_WriteChar(UART0_P, store);
        delay(500000);
    }
    
    return 0;
}
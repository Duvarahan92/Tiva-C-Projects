#include <stdint.h>
#include <stdio.h>
#include "tm4c123GH6PM_spi_driver.h"
#include "tm4c123GH6PM_gpio_driver.h"
#include "tm4c123GH6PM_uart_driver.h"

void delay(uint32_t ticks) 
{
  for (size_t i = 0; i < ticks; i++);
}

int main(void)
{
    uint32_t SysClk = 16000000;
    uint32_t BaudRate = 115200;
    uint32_t BitRate = 1000000;
    unsigned char word[] = "The definition of insanity is doing the same thing over and over again and expecting different results.";
    uint8_t store;

    // Activate clock for UART_0 and SPI module and GPIO port A
    UART_EnableClk(SYSCTL_RCGCUART_R0);
    SSI_EnableClk(SYSCTL_RCGCSSI_R0);
    GPIO_EnableClk(SYSCTL_RCGCGPIO_R0);

    // Configure for GPIO A:
    // Pin 2 as clk
    // Pin 4 as MISO
    // Pin 5 as MOSO
    GPIO_SSIType(GPIOA_P, GPIO_PIN_2, GPIO_PCTL_PA2_SSI0CLK);
    GPIO_SSIType(GPIOA_P, GPIO_PIN_4, GPIO_PCTL_PA4_SSI0RX);
    GPIO_SSIType(GPIOA_P, GPIO_PIN_5, GPIO_PCTL_PA5_SSI0TX);

    //Configure pin 0 and 1 on GPIO port A as UART
    GPIO_UARTType(GPIOA_P, GPIO_PIN_0, GPIO_PCTL_PA0_U0RX);
    GPIO_UARTType(GPIOA_P, GPIO_PIN_1, GPIO_PCTL_PA1_U0TX);

    // Configure SSI_0 module
    
    // Disable SSI_0 module
    SSI_DisableModule(SSI0_P);
    
    // Select master mode
    SET_SSIMode(SSI0_P, SSI_CR1_M);

    // Configure SPI clock
    SSI_ConfigClk(SSI0_P, SysClk, BitRate);

    // Configure SSI_0 module
    SSI_ConfigModule(SSI0_P, SSI_CR0_MODE_0, SSI_CR0_FRF_MOTO, SSI_CR0_DSS_8);

    // Enable loopback
    EnableLoopbackMode(SSI0_P);

    // Enable SSI_0 module
    SSI_EnableModule(SSI0_P);

    //Config UART 0
    // 16 MHz clock, 115200 baud rate, no parity, 8 bit word length and
    // one stop bit
    UART_ConfigModule(UART0_P, SysClk, BaudRate, UART_LCRH_NONE, UART_LCRH_WLEN_8, UART_LCRH_STP1);

    // Write-read SPI
    for (int i = 0; i < sizeof(word); i++)
    {
        SPI_SendData(SSI0_P, word[i]);
        SPI_ReceiveData(SSI0_P, &store);
        UART_WriteChar(UART0_P, store);
        delay(500000);
    }
    
    return 0;
}
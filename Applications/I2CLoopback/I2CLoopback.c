#include <stdint.h>
#include <stdio.h>
#include "tm4c123GH6PM_i2c_driver.h"
#include "tm4c123GH6PM_gpio_driver.h"
#include "tm4c123GH6PM_uart_driver.h"

int main(void)
{
    static const uint32_t SysClk = 16000000;
    static const uint32_t BaudRate = 115200;
    static const uint8_t SlaveAddr = 59;
    static const unsigned char word[]= "Hello World\r\n";
    static uint8_t store;

    //Config UART 0
    // 16 MHz clock, 115200 baud rate, no parity, 8 bit word length and
    // one stop bit
    UART_Init(UART0_P, SysClk, BaudRate, UART_LCRH_NONE, UART_LCRH_WLEN_8, UART_LCRH_STP1);

    //Init master and slave
    I2C_MasterInit(I2C0_P, STANDARD_MODE, SysClk);
    I2C_SlaveInit(I2C0_P, SlaveAddr);

    I2C_EnableLoopBack(I2C0_P);

    //Set slave address of the master
    I2C_SetMasterSlaveAddr(I2C0_P, SlaveAddr, 0);

    //Write-read I2C Master -> Slave
    UART_WriteString(UART0_P, "Master -> Slave\r\n");

    for (int i = 0; i < sizeof(word); i++) {
        I2C_MasterSendData(I2C0_P, word[i]);
        I2C_MasterCTRL(I2C0_P, SINGLE_SEND);

        //Wait until slave has received data
        while(!(I2C_SlaveReceiveBusy(I2C0_P)));

        //Slave receive data
        store = I2C_SlaveReceiveData(I2C0_P);

        //wait until master is idle
        while(I2C_MasterBusy(I2C0_P));

        UART_WriteChar(UART0_P, store);
    }

    return 0;
}
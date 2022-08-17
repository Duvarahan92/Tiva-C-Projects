#include <stdio.h>
#include "Serial_Interface.h"
#include "tm4c123GH6PM_uart_driver.h"
#include "BME680.h"

int main ()
{
    static const uint8_t addr = 0x77;
    static uint8_t chip_id[3];
    static uint8_t chip_id_reg = 0x8A;

    Serial_InterfaceInit(addr);
    I2C_Read(chip_id_reg, chip_id, 6);

    for(uint8_t i = 0; i < 6; i++)
      {  UART_WriteHex(UART0_P, chip_id[i]);
        UART_WriteString(UART0_P, "\r\n");
        }

    return 0;
}
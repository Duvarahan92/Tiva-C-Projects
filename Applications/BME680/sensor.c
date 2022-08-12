#include <stdio.h>
#include "Serial_Interface.h"
#include "tm4c123GH6PM_uart_driver.h"
#include "BME680.h"

int main ()
{
    static uint8_t chip_id;
    static uint8_t chip_id_reg = 0xD0;

    Serial_InterfaceInit();

    I2C_Read(chip_id_reg, &chip_id, 1);
    UART_WriteHex(UART0_P, chip_id);

    return 0;
}
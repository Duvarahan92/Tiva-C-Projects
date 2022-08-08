/*Serial.h*/
#include <stdint.h>
#include <stdio.h>

#ifndef INC_Serial_H_
#define INC_Serial_H_

/*********************************************************************************
*                        User interface functions
*
**********************************************************************************/
/*
 * Peripherals setup
 *
 */

void Serial_InterfaceInit();
void Serial_InterfaceDeInit();

 /*
 * Data read and write
 *
 */
void I2C_Write(uint8_t *reg_addr, uint8_t *reg_data, uint8_t len);
void I2C_Read(uint8_t reg_addr, uint8_t *reg_data, uint8_t len);

 /*
 * Other functions
 *
 */
void delay(uint32_t ticks);

#endif  // __Serial_H__
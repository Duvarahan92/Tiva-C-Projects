/*tm4c123GH6PM_i2c_driver.h*/
#include <stdint.h>

#ifndef INC_TM4C123GH6PM_I2C_DRIVER_H_
#define INC_TM4C123GH6PM_I2C_DRIVER_H_

/*********************************************************************************
*                        Macros used to config registers
*
**********************************************************************************/

 enum I2C_Module
 {
   I2C0_P = 0, 
   I2C1_P = 1,
   I2C2_P = 2,
   I2C3_P = 3
 };

 /* 
 * Interrupt assignments 
 * 
 */
#define INT_I2C0                8           // I2C0
#define INT_I2C1                37          // I2C1
#define INT_I2C2                68          // I2C2
#define INT_I2C3                69          // I2C3

/* 
 * I2C module run mode clock gating control
 * Arguments to I2C_EnableClk and I2C_DisableClk functions
 */
#define SYSCTL_RCGCI2C_R3       0x00000008  // I2C Module 3 Run Mode Clock Gating Control
#define SYSCTL_RCGCI2C_R2       0x00000004  // I2C Module 2 Run Mode Clock Gating Control
#define SYSCTL_RCGCI2C_R1       0x00000002  // I2C Module 1 Run Mode Clock Gating Control
#define SYSCTL_RCGCI2C_R0       0x00000001  // I2C Module 0 Run Mode Clock Gating Control

/*********************************************************************************
*                        API supported by this driver
*
**********************************************************************************/
/*
 * Peripheral setup
 *
 */
void I2C_EnableClk(uint8_t SYSCTL_RCGCI2C_MODULE);
void I2C_DisableClk(uint8_t SYSCTL_RCGCI2C_MODULE);
void I2C_EnableMaster(uint8_t I2Cx);
void I2C_DisableMaster(uint8_t I2Cx);
void I2C_EnableSlave(uint8_t I2Cx);
void I2C_DisableSlave(uint8_t I2Cx);

 /*
 * Helping macros
 *
 */
 #define ENABLE  1
 #define DISABLE 0

#endif  // __TM4C123GH6PM_GPIO_DRIVER_H__
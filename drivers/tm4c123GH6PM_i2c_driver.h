/*tm4c123GH6PM_i2c_driver.h*/
#include <stdint.h>
#include <stdio.h>

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
 * I2C module speed mode
 * Arguments to I2C_MasterInit
 */
#define STANDARD_MODE           100000      // 100 Kbps
#define FAST_MODE               400000      // 400 Kbps
#define FAST_MODE_PLUS          1000000     // 400 Kbps

/* 
 * I2C module recevie or send
 * Arguments to I2C_SetMasterSlaveAddr
 */
#define RECEIVE                 1           // Recieve
#define SEND                    0           // Send  

/* 
 * I2C module master control 
 * Arguments to I2C_MasterCTRL
 */
#define SINGLE_SEND             0x00000007  // Send only once and stop
#define SINGLE_RECEIVE          0x00000007  // Receive only once and stop
#define BURST_SEND_START        0x00000003  // Start sending without stop
#define BURST_SEND_CONT         0x00000001  // Continue sending without stop
#define BURST_SEND_FINNISH      0x00000005  // Finish sending after last byte
#define BURST_SEND_STOP         0x00000004  // Stop sending
#define BURST_RECEIVE_START     0x0000000b  // Start receiving without stop
#define BURST_RECEIVE_CONT      0x00000009  // Continue receiving without stop
#define BURST_RECEIVE_FINNISH   0x00000005  // Finnish receiving after last byte
#define BURST_RECEIVE_STOP      0x00000004  // Stop receiving
#define HS_SEND                 0x00000013  // Send in high-speed mode

/*********************************************************************************
*                        API supported by this driver
*
**********************************************************************************/
/*
 * Peripheral setup
 *
 */
//void I2C_EnableClk(uint8_t SYSCTL_RCGCI2C_MODULE);
//void I2C_DisableClk(uint8_t SYSCTL_RCGCI2C_MODULE);
void I2C_Init(uint8_t I2Cx);
void I2C_DeInit(uint8_t I2Cx);
void I2C_EnableLoopBack(uint8_t I2Cx);
void I2C_DisableLoopBack(uint8_t I2Cx);

/*
 * Peripheral configure
 *
 */
void I2C_MasterInit(uint8_t I2Cx, uint32_t Speed_Mode, uint32_t Clk);
void I2C_MasterDeInit(uint8_t I2Cx);
void I2C_SlaveInit(uint8_t I2Cx, uint8_t SlaveAddr);
void I2C_SlaveDeInit(uint8_t I2Cx);
void I2C_MasterCTRL(uint8_t I2Cx, uint8_t CtrlCmd);

 /*
 * Data read and write
 *
 */
void I2C_SetMasterSlaveAddr(uint8_t I2Cx, uint8_t SlaveAddr, uint8_t RS);
void I2C_MasterSendData(uint8_t I2Cx, uint8_t Data);
uint8_t I2C_MasterReceiveData(uint8_t I2Cx);
void I2c_SlaveSendData(uint8_t I2Cx, uint8_t Data);
uint8_t I2C_SlaveReceiveData(uint8_t I2Cx);


 /*
 * Other functions
 *
 */
uint8_t I2C_SignalLineStatus(uint8_t I2Cx);
uint8_t I2C_MasterBusy(uint8_t I2Cx);
uint8_t I2C_SlaveReceiveBusy(uint8_t I2Cx);
uint32_t I2C_MasterGetErrorStatus(uint8_t I2Cx);

 /*
 * Helping macros
 *
 */
 #define ENABLE  1
 #define DISABLE 0

#endif  // __TM4C123GH6PM_GPIO_DRIVER_H__
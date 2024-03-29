#include "Serial_Interface.h"
#include "tm4c123GH6PM_i2c_driver.h"
#include "tm4c123GH6PM_uart_driver.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/
static const uint32_t SysClk = 16000000;
static const uint32_t BaudRate = 115200;
static uint8_t SlaveAddr;

/*********************************************************************************
*                        User interface functions
*
**********************************************************************************/

/********************************************************************************
 * @fn                     - Serial_InterfaceInit
 *
 * @brief                  - This function initialize the necessary serial interfaces needed to communicate
 *                         - with the sensor.
 * 
 * @param[in]              - Slave address for I2C communication. For SPI 0x00 can be sent. 
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 void Serial_InterfaceInit(uint8_t addr)
 {
   SlaveAddr = addr;
   //Config UART 0
   // 16 MHz clock, 115200 baud rate, no parity, 8 bit word length and
   // one stop bit
    UART_Init(UART0_P, SysClk, BaudRate, UART_LCRH_NONE, UART_LCRH_WLEN_8, UART_LCRH_STP1);

    I2C_MasterInit(I2C0_P, STANDARD_MODE, SysClk);
    
    UART_WriteString(UART0_P, "Serial connection establised\r\n");
    UART_WriteString(UART0_P, "I2C Interface\r\n");
 }

 /********************************************************************************
 * @fn                     - Serial_interfaceDeinit
 *
 * @brief                  - This function deinitialize serial interfaces
 * 
 * @param[in]              - none
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 void Serial_InterfaceDeInit()
 {
   UART_WriteString(UART0_P, "Disconnecting serial interface\r\n");
   UART_Deinit(UART0_P);
   I2C_MasterDeinit(I2C0_P);
 }

 /********************************************************************************
 * @fn                     - I2C_Write
 *
 * @brief                  - Sends data via I2C
 * 
 * @param[in]              - Array of register to be written too
 * @param[in]              - Array of data that shall be written to the corresponding register.
 * @param[in]              - Length of both array
 * 
 * @return                 - none
 * 
 * @Note                   - Writing is done by sending the slave address in write mode.
 *                         - The transaction is ended by a stop condition.
 */

 void I2C_Write(uint8_t *reg_addr, uint8_t *reg_data, uint8_t len)
 {
   uint8_t i;
   I2C_SetMasterSlaveAddr(I2C0_P, SlaveAddr, 0);

   for(i = 0; i < len; i++) {
      I2C_MasterSendData(I2C0_P, reg_addr[i]);
      I2C_MasterCTRL(I2C0_P, (i==0 ? BURST_SEND_START : BURST_SEND_CONT));
      I2C_MasterSendData(I2C0_P, reg_data[i]);
      I2C_MasterCTRL(I2C0_P, BURST_SEND_CONT);
   } 

   I2C_MasterCTRL(I2C0_P, BURST_SEND_FINNISH);
 }

  /********************************************************************************
 * @fn                     - I2C_Read
 *
 * @brief                  - Receive data via I2C
 * 
 * @param[in]              - Array of register to be read from
 * @param[in]              - Array of data that shall store the data being read.
 * @param[in]              - Length of both array
 * 
 * @return                 - none
 * 
 * @Note                   - To be able to read registers, first the registe address must be sent in write mode.
 *                         - Then either a stop or a repeated start condition must be generated.
 *                         - After this the slave is addressed in read mode, after which the slave
 *                         - sends out data from auto-incremented register address untill a NOACKM and stop condiition occurs.
 *
 */

 void I2C_Read(uint8_t reg_addr, uint8_t *reg_data, uint8_t len) 
 {
   uint8_t i;
   I2C_SetMasterSlaveAddr(I2C0_P, SlaveAddr, 0);
   I2C_MasterSendData(I2C0_P, reg_addr);
   I2C_MasterCTRL(I2C0_P, BURST_SEND_START);
   I2C_SetMasterSlaveAddr(I2C0_P, SlaveAddr, 1);
   I2C_MasterCTRL(I2C0_P, BURST_RECEIVE_START);

   for(i = 0; i < len; i++) { 
      reg_data[i] = I2C_MasterReceiveData(I2C0_P);
      I2C_MasterCTRL(I2C0_P, (i == (len - 1) ? BURST_RECEIVE_FINNISH : BURST_RECEIVE_CONT));
    }
}

void delay(uint32_t ticks) 
{
  for (size_t i = 0; i < ticks; i++);
}
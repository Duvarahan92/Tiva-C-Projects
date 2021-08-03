#include "tm4c123GH6PM_uart_driver.h"
#include "tm4c123gh6pm.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/

static UART_RegDef_t* MapUARTBaseAddress[8] =
{
    UART0, 
    UART1,
    UART2,
    UART3,
    UART4,
    UART5,
    UART6,
    UART7
};

/********************************************************************************
 * @fn                     - UART_Get_Module
 *
 * @brief                  - This function gets the struct ptr to a module
 * 
 * @param[in]              - UART Module
 * 
 * @return                 - Struct ptr to a module
 * 
 * @Note                   - none
 */
 static UART_RegDef_t* UART_Get_Module(uint8_t UART_Module)
 {
     return MapUARTBaseAddress[UART_Module];
 }

/*********************************************************************************
*                           API functions
*
**********************************************************************************/ 

/********************************************************************************
 * @fn                     - UART_EnableClk
 *
 * @brief                  - This function enables peripheral clock for the given UART module
 * 
 * @param[in]              - Run mode clock gating control
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 void UART_EnableClk(uint8_t SYSCTL_RCGCUART_MODULE)
 {
     
    SYSCTL -> RCGCUART |= SYSCTL_RCGCUART_MODULE;
 }

 /********************************************************************************
 * @fn                     - UART_DisableClk
 *
 * @brief                  - This function disables peripheral clock for the given UART module
 * 
 * @param[in]              - Run mode clock gating control
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 void UART_DisableClk(uint8_t SYSCTL_RCGCUART_MODULE)
 {
     
    SYSCTL -> RCGCUART &= ~(SYSCTL_RCGCUART_MODULE);
 }

 /********************************************************************************
 * @fn                     - UART_EnableModule
 *
 * @brief                  - This function enables UART module
 * 
 * @param[in]              - UART module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 void UART_EnableModule(uint8_t UARTx)
 {
    UART_RegDef_t *pUART = UART_Get_Module(UARTx);

    // Enable FIFO
    UART_FIFOEnable(UARTx);

    // Enable RX, TX and UART 
    pUART -> UARTCTL |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
 }

 /********************************************************************************
 * @fn                     - UART_DisableModule
 *
 * @brief                  - This function disables UART module
 * 
 * @param[in]              - UART module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 void UART_DisableModule(uint8_t UARTx)
 {
   UART_RegDef_t *pUART = UART_Get_Module(UARTx);

   // Wait for end of TX
   while (pUART -> UARTFR & UART_FR_BUSY);

   // Disable FIFO
   UART_FIFODisable(UARTx);

   // Disable TX , RX and UART  
   pUART -> UARTCTL &= ~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
 }

  /********************************************************************************
 * @fn                     - UART_Reset
 *
 * @brief                  - This function reset the UART module
 * 
 * @param[in]              - UART port
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void UART_Reset(uint8_t SYSCTL_SRUART)
{
    SYSCTL -> SRUART |= SYSCTL_SRUART;
    SYSCTL -> SRUART &= ~(SYSCTL_SRUART);

}

  /********************************************************************************
 * @fn                     - UART_FIFOEnable
 *
 * @brief                  - This function enables FIFO
 * 
 * @param[in]              - UART port
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void UART_FIFOEnable(uint8_t UARTx)
{
   UART_RegDef_t *pUART = UART_Get_Module(UARTx);
   pUART -> UARTLCRH |= UART_LCRH_FEN;
}

/********************************************************************************
 * @fn                     - UART_FIFOEnable
 *
 * @brief                  - This function enables FIFO
 * 
 * @param[in]              - UART port
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void UART_FIFODisable(uint8_t UARTx)
{
   UART_RegDef_t *pUART = UART_Get_Module(UARTx);
   pUART -> UARTLCRH &= ~(UART_LCRH_FEN);
}

  /********************************************************************************
 * @fn                     - UART_ConfigModule
 *
 * @brief                  - This function configure the UART module
 * 
 * @param[in]              - UART port
 * @param[in]              - System clock
 * @param[in]              - Desired baud rate
 * @param[in]              - Parity
 * @param[in]              - Data length (bits)
 * @param[in]              - Stop bits
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void UART_ConfigModule(uint8_t UARTx, uint32_t Clk, uint32_t Baud, uint32_t Parity, uint32_t DataLength, uint32_t StopBits)
{
   UART_RegDef_t *pUART = UART_Get_Module(UARTx);
   uint32_t div;

   // Stop the UART module
   UART_DisableModule(UARTx);

   // Compute the fractional baud rate
   div = (((Clk * 8) / Baud) + 1) / 2;

   // Set baud rate
   pUART -> UARTBRD = div / 64;
   pUART -> UARTFBRD = div % 64;

   // Set parity
   pUART -> UARTLCRH |= Parity;
   pUART -> UARTLCRH |= DataLength;
   pUART -> UARTLCRH |= StopBits;

   // Clear flags
   pUART -> UARTFR = 0;

   // Start the UART module
   UART_EnableModule(UARTx);
}

/********************************************************************************
 * @fn                     - UART_WriteChar
 *
 * @brief                  - Write a character to the transmit FIFO
 * 
 * @param[in]              - UART port
 * @param[in]              - Character to be written
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void UART_WriteChar(uint8_t UARTx, unsigned char data)
 {
    UART_RegDef_t *pUART = UART_Get_Module(UARTx);

    // Waits until there is space in the FIFO
    while(pUART -> UARTFR & UART_FR_TXFF);

    // Put character in FIFO
    pUART -> UARTDR = data;
 }

 /********************************************************************************
 * @fn                     - UART_WriteString
 *
 * @brief                  - Write a string to the transmit FIFO
 * 
 * @param[in]              - UART port
 * @param[in]              - String to be written. Use NULL termination to end string.
 * 
 * @return                 - none
 * 
 * @Note                   - Use NULL termination to end string.
 */
void UART_WriteString(uint8_t UARTx, unsigned char buffer[])
{
   uint32_t i = 0;

   // Loop through the buffer and write out each char
   while (buffer[i] != '\0')
   {
      UART_WriteChar(UARTx, buffer[i]);
      i++;
   }   
}

 /********************************************************************************
 * @fn                     -UART_ReadChar
 *
 * @brief                  - Read a charcter from receive FIFO
 * 
 * @param[in]              - UART port
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 uint8_t UART_ReadChar(uint8_t UARTx)
 {
    UART_RegDef_t *pUART = UART_Get_Module(UARTx);

    // Wait until FIFO is not empty
    while (pUART -> UARTFR & UART_FR_RXFE);

    // return char
    return(pUART -> UARTDR);
 }
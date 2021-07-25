#include "tm4c123GH6PM_spi_driver.h"
#include "tm4c123gh6pm.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/

static SSI_RegDef_t* MapSSIBaseAddress[4] =
{
    SSI0, 
    SSI1,
    SSI2,
    SSI3
};

/********************************************************************************
 * @fn                     - SSI_Get_Module
 *
 * @brief                  - This function gets the struct ptr to a module
 * 
 * @param[in]              - SSI Module
 * 
 * @return                 - Struct ptr to a module
 * 
 * @Note                   - none
 */
 static SSI_RegDef_t* SSI_Get_Module(uint8_t SSI_Module)
 {
     return MapSSIBaseAddress[SSI_Module];
 }

 /********************************************************************************
 * @fn                     - SSI_Check_Module
 *
 * @brief                  - This function check if SSI module is valid
 * 
 * @param[in]              - SSI module
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */

static uint8_t SSI_Check_Module(uint8_t SSI_Module)
{
    for (int i = SSI0_P; i <= SSI3_P; i++)
    {
        if (i == SSI_Module)
            return TRUE;
    }

    return FALSE;
}

/********************************************************************************
 * @fn                     - SSI_EnableClk
 *
 * @brief                  - This function enables peripheral clock for the given SSI module
 * 
 * @param[in]              - Run mode clock gating control
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 /*********************************************************************************
*                           API functions
*
**********************************************************************************/ 

 void SSI_EnableClk(uint8_t SYSCTL_RCGCSSI_MODULE)
 {
     
     SYSCTL_RCGCSSI_MODULE |= SYSCTL_RCGCSSI_MODULE;
 }

 /********************************************************************************
 * @fn                     - SSI_DisableClk
 *
 * @brief                  - This function disables peripheral clock for the given SSI module
 * 
 * @param[in]              - Run mode clock gating control
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSI_DisableClk(uint8_t SYSCTL_RCGCSSI_MODULE)
 {
     
     SYSCTL_RCGCSSI_MODULE &= ~(SYSCTL_RCGCSSI_MODULE);
 }


 /********************************************************************************
 * @fn                     - SSI_Enable
 *
 * @brief                  - This function enables the SSI module
 * 
 * @param[in]              - SSI module which need to be enabled
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void SSI_Enable(uint8_t SSIx)
{
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);
    pSSI -> SSICR1 |= SSI_CR1_SSE;
}

 /********************************************************************************
 * @fn                     - SSI_Disable
 *
 * @brief                  - This function disables the SSI module
 * 
 * @param[in]              - SSI module which need to be disabled
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSI_Disable(uint8_t SSIx)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);
    pSSI -> SSICR1 &= ~(SSI_CR1_SSE);
 }

/********************************************************************************
 * @fn                     - SET_SSIMode
 *
 * @brief                  - This function SSI module as master or slave
 * 
 * @param[in]              - SSI module which need to be set
 * @param[in]              - Master mode or slave mode
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SET_SSIMode(uint8_t SSIx, uint32_t Mode)
 {
     SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     if (Mode == SSI_CR1_M)
        pSSI -> SSICR1 &= ~(SSI_CR1_S); 
     pSSI -> SSICR1 |= Mode;
 }

 /********************************************************************************
 * @fn                     - SSI_ConfigClk
 *
 * @brief                  - This function configure the SSI clock
 * 
 * @param[in]              - SSI module which need to be configured
 * @param[in]              - System clock
 * @param[in]              - defines the bit rate for the SSI.  This bit
 *                         - rate must satisfy the following clock ratio criteria:
 *                         - FSSI >= 2 * bit rate (master mode)
 *                         - FSSI >= 12 * bit rate (slave modes)
 *                         -  where FSSI is the frequency of the clock supplied to the SSI module.
 *
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSI_ConfigClk(uint8_t SSIx, uint32_t SSIClk, uint32_t BitRate)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     uint32_t MaxBitRate;
     uint32_t PreDiv;
     uint32_t SCR;

     // Set the clock predivider and clock rate
    MaxBitRate = SSIClk / BitRate;
    PreDiv = 0;

    do 
    {
        PreDiv += 2;
        SCR = (MaxBitRate / PreDiv) - 1;
    }
    while (SCR > 255);

    pSSI -> SSICPSR = PreDiv;
    pSSI -> SSICR0 = SCR << 8;
}

 /********************************************************************************
 * @fn                     - SSI_ConfigModule
 *
 * @brief                  - This function configure the SSI module
 * 
 * @param[in]              - SSI module which need to be configured
 * @param[in]              - Configure the SSI clock phase and polarity
 * @param[in]              - Configure SSI frame format
 * @param[in]              - Configure SSI data size
 *
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSI_ConfigModule(uint8_t SSIx, uint32_t PhasePolMode, uint32_t ProtocolMode, uint32_t DSS)
 {
     SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     // Set phase and polarity
     pSSI -> SSICR0 |= PhasePolMode;

     // Set protocol
     pSSI -> SSICR0 |= ProtocolMode;

     // Set data size
     pSSI -> SSICR0 |= DSS;
 }
 
/********************************************************************************
 * @fn                     - SPI_SendData
 *
 * @brief                  - This function sends data while blocking
 * 
 * @param[in]              - SSI module
 * @param[in]              - Data to send
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void SPI_SendData(uint8_t SSIx, uint16_t Data)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

    // Wait untill there is space in the transmit FIFO
    while (!(pSSI -> SSISR & SSI_SR_TNF));

    // Write data to the FIFO
    pSSI -> SSIDR = Data;
 }

 /********************************************************************************
 * @fn                     - SPI_ReceiveData
 *
 * @brief                  - This function reads data while blocking
 * 
 * @param[in]              - SSI module
 * @param[in]              - pointer to the place where Ddata will get stored
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void SPI_ReceiveData(uint8_t SSIx, uint16_t *Data)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

    // Wait untill there is space in the transmit FIFO
    while (!(pSSI -> SSISR & SSI_SR_RNE));

    // Read data from the FIFO
   *Data = pSSI -> SSIDR;
 }

 /********************************************************************************
 * @fn                     - EnableLoopbackMode
 *
 * @brief                  - This function enables loopback for the SSI module
 * 
 * @param[in]              - SSI module which will be in loopback mode
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void EnableLoopbackMode(uint8_t SSIx)
 {
     SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     pSSI -> SSICR1 |= SSI_CR1_LBM;
 }

 /********************************************************************************
 * @fn                     - DisableLoopbackMode
 *
 * @brief                  - This function disables loopback for the SSI module
 * 
 * @param[in]              - SSI module which will be in normal mode
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void DisableLoopbackMode(uint8_t SSIx)
 {
     SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     pSSI -> SSICR1 &= ~(SSI_CR1_LBM);
 }

 /********************************************************************************
 * @fn                     - SSI_Busy
 *
 * @brief                  - SSI is currently transmitting and/or receiving a frame, or
 *                         - the transmit FIFO is not empty
 * 
 * @param[in]              - SSI module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 uint8_t SSI_Busy(uint8_t SSIx)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

    return((pSSI -> SSISR & SSI_SR_BSY) ? TRUE : FALSE);
 }


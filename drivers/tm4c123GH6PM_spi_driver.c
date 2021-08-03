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

/*********************************************************************************
*                           API functions
*
**********************************************************************************/ 

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

 void SSI_EnableClk(uint8_t SYSCTL_RCGCSSI_MODULE)
 {
     
    SYSCTL -> RCGCSSI |= SYSCTL_RCGCSSI_MODULE;
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
     
    SYSCTL -> RCGCSSI &= ~(SYSCTL_RCGCSSI_MODULE);
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
void SSI_EnableModule(uint8_t SSIx)
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
 void SSI_DisableModule(uint8_t SSIx)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);
    pSSI -> SSICR1 &= ~(SSI_CR1_SSE);
 }

 /********************************************************************************
 * @fn                     - SSI_Reset
 *
 * @brief                  - This function reset the SSI module
 * 
 * @param[in]              - SSI port
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void SSI_Reset(uint8_t SYSCTL_SRSSI)
{
    SYSCTL -> SRSSI |= SYSCTL_SRSSI;
    SYSCTL -> SRSSI &= ~(SYSCTL_SRSSI);

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
void SPI_SendData(uint8_t SSIx, uint8_t Data)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

    // Wait untill there is space in the transmit FIFO
    while (!(pSSI -> SSISR & SSI_SR_TNF));

    // Write data to the FIFO
    pSSI -> SSIDR = Data;
 }

/********************************************************************************
 * @fn                     - SPI_SendNonBlockingData
 *
 * @brief                  - This function sends data 
 * 
 * @param[in]              - SSI module
 * @param[in]              - Data to send
 * 
 * @return                 - TRUE or FALSE
 * 
 * @Note                   - Non Blocking
 */
 uint8_t SPI_SendNonBlockingData(uint8_t SSIx, uint8_t Data)
 {
      SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     // Checks if the transmit FIFO is not full
     if (pSSI -> SSISR & SSI_SR_TNF)
     {
        // Write data to the FIFO
        pSSI -> SSIDR = Data;
        return TRUE;
     }

     return FALSE;
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
void SPI_ReceiveData(uint8_t SSIx, uint8_t *Data)
 {
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

    // Wait untill there is space in the receive FIFO
    while (!(pSSI -> SSISR & SSI_SR_RNE));

    // Read data from the FIFO
   *Data = pSSI -> SSIDR;
 }

  /********************************************************************************
 * @fn                     - SPI_ReceiveNonBlockingData
 *
 * @brief                  - This function reads data
 * 
 * @param[in]              - SSI module
 * @param[in]              - pointer to the place where Ddata will get stored
 * 
 * @return                 - TRUE or FALSE
 * 
 * @Note                   - Non Blocking
 */
 uint8_t SPI_ReceiveNonBlockingData(uint8_t SSIx, uint8_t *Data)
 {
     SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     // Check if receive FIFO is not empty
     if (pSSI -> SSISR & SSI_SR_RNE) 
     {
         *Data = pSSI -> SSIDR;
         return TRUE;
     }

     return FALSE;
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
 * @fn                     - SSI_EnableInterrupt
 *
 * @brief                  - This function enables interrupt
 * @param[in]              - SSI module
 * @param[in]              - Interrupt to disable
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSI_EnableInterrupt(uint8_t SSIx, uint8_t InterruptMask)
 {
     SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     pSSI -> SSIIM |= InterruptMask;
 }

   /********************************************************************************
 * @fn                     - SSI_DisableInterrupt
 *
 * @brief                  - This function disables interrupt
 * @param[in]              - SSI module
 * @param[in]              - Interrupt to disable
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSI_DisableInterrupt(uint8_t SSIx, uint8_t InterruptMask)
 {
     SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

     pSSI -> SSIIM &= ~(InterruptMask);
 }


 /********************************************************************************
 * @fn                     - SSI_ClearInterrupt
 *
 * @brief                  - This function can clear Time-Out- and Overrun interrupt
 * @param[in]              - SSI module
 * @param[in]              - Either Time-out or OVerrun interrupt
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void SSI_ClearInterrupt(uint8_t SSIx, uint8_t Interrupt)
{
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);

    pSSI -> SSIICR |= Interrupt;
}

 /********************************************************************************
 * @fn                     - SSI_GetInterruptStatus
 *
 * @brief                  - Gets raw interrupt status
 * @param[in]              - SSI module
 * 
 * @return                 - Raw interrupts status
 * 
 * @Note                   - none
 */
uint32_t SSI_GetInterruptStatus(uint8_t SSIx)
{
    SSI_RegDef_t *pSSI = SSI_Get_Module(SSIx);
    
    return(pSSI -> SSIRIS);
}


 /********************************************************************************
 * @fn                     - SSI_IRQConfig
 *
 * @brief                  - This function enbales or disables interrupt from processor side
 * @param[in]              - IRQ number
 * @param[in]              - ENABLE or DISABLE macros
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void SSI_IRQConfig(uint8_t IRQn, uint8_t Ctrl)
{
    if (Ctrl == ENABLE)
    {
        if (IRQn <= 31)
        {
            NVIC_EN0_R |= (1 << IRQn);
        }else if (IRQn > 31 && IRQn < 64)
        {
            NVIC_EN1_R |= (1 << (IRQn % 32));
        }
    }else
    {
        if (IRQn <= 31)
        {
            NVIC_DIS0_R |= (1 << IRQn);
        }else if (IRQn > 31 && IRQn < 64)
        {
            NVIC_DIS1_R |= (1 << (IRQn % 32));
        }
    }
}

/********************************************************************************
 * @fn                     - SSI_IRQPriorityConfig
 *
 * @brief                  - This function sets interrupt priority from processor side
 * 
 * @param[in]              - IRQ number
 * @param[in]              - IRQ priority
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void SSI_IRQPriorityConfig(uint8_t IRQn, uint32_t IRQPriority)
{
    uint8_t iprx = IRQn / 4;
    uint8_t sections = IRQn % 4;

    uint8_t Shift_amount = (8 * sections) + 5;
    *(NVIC_PRI0_R + iprx) = (IRQPriority << Shift_amount);
}

/********************************************************************************
 * @fn                     - SSI_IRQHandling
 *
 * @brief                  - Handles different interrupt triggers
 * 
 * @param[in]              - SSI module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void SSI_IRQHandling(uint8_t SSIx, uint8_t TXData, uint8_t *RXData)
{
   /* uint32_t interruptStatus = SSI_GetInterruptStatus(SSIx);

    if (interruptStatus == SSI_RIS_TXRIS)
        while (SPI_SendNonBlockingData(SSIx, TXData));
    else if (interruptStatus == SSI_IM_RXIM)
        SSI_ClearInterrupt(SSIx,SSI_ICR_RORIC);
    else if (interruptStatus == SSI_RIS_RTRIS)
        SSI_ClearInterrupt(SSIx, SSI_ICR_RTIC);
    else */
        while (SPI_ReceiveNonBlockingData(SSIx, RXData));

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


#include "tm4c123GH6PM_spi_driver.h"

/********************************************************************************
 * @fn                     - SSI_CLK_CTRL
 *
 * @brief                  - This function enbales or disables peripheral clock for the given SSI module
 * 
 * @param[in]              - Run mode clock gating control
 * @param[in]              - ENABLE or DISABLE macros
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSI_CLK_CTRL(uint32_t SYSCTL_RCGCSSI_MODULE, uint8_t Ctrl)
 {
     if (Ctrl == ENABLE)
        SYSCTL -> RCGCSSI |= SYSCTL_RCGCSSI_MODULE;
     
     SYSCTL_RCGCSSI_MODULE &= ~(SYSCTL_RCGCSSI_MODULE);
 }

 /********************************************************************************
 * @fn                     - SSIEnable
 *
 * @brief                  - This function enables the SSI module
 * 
 * @param[in]              - SSI module which need to be enabled
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void SSIEnable(SSI_RegDef_t *pSSIx)
{
    pSSIx -> SSICR1 |= SSI_CR1_SSE;
}

 /********************************************************************************
 * @fn                     - SSIDisable
 *
 * @brief                  - This function disables the SSI module
 * 
 * @param[in]              - SSI module which need to be disabled
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void SSIDisable(SSI_RegDef_t *pSSIx)
 {
      pSSIx -> SSICR1 &= ~(SSI_CR1_SSE);
 }

/********************************************************************************
 * @fn                     - SET_SSI_MODE
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
 void SET_SSI_MODE(SSI_RegDef_t *pSSIx, uint32_t Mode)
 {
     if (Mode == SSI_CR1_M)
        pSSIx -> SSICR1 &= ~(SSI_CR1_S); 
     pSSIx -> SSICR1 |= Mode;
 }

 /********************************************************************************
 * @fn                     - ConfigSSIClk
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
 void ConfigSSIClk(SSI_RegDef_t *pSSIx, uint32_t SSIClk, uint32_t BitRate)
 {
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

    pSSIx -> SSICPSR = PreDiv;
    pSSIx -> SSICR0 = SCR << 8;
}

 /********************************************************************************
 * @fn                     - ConfigSSIModule
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
 void ConfigSSIModule(SSI_RegDef_t *pSSIx, uint32_t PhasePolMode, uint32_t ProtocolMode, uint32_t DSS)
 {
     // Set phase and polarity
     pSSIx -> SSICR0 |= PhasePolMode << 6;

     // Set protocol
     pSSIx -> SSICR0 |= ProtocolMode;

     // Set data size
     pSSIx -> SSICR0 |= DSS;
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
 void EnableLoopbackMode(SSI_RegDef_t *pSSIx)
 {
     pSSIx -> SSICR1 |= SSI_CR1_LBM;
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
 void DisableLoopbackMode(SSI_RegDef_t *pSSIx)
 {
     pSSIx -> SSICR1 &= ~(SSI_CR1_LBM);
 }

/*********************************************************************************
*                        Internal functions
*
**********************************************************************************/

#include "tm4c123GH6PM_i2c_driver.h"
#include "tm4c123gh6pm.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/

static I2C_RegDef_t* MapI2CBaseAddress[4] =
{
    I2C0, 
    I2C1,
    I2C2,
    I2C3
};

/********************************************************************************
 * @fn                     - I2C_Get_Module
 *
 * @brief                  - This function gets the struct ptr to a module
 * 
 * @param[in]              - I2C Module
 * 
 * @return                 - Struct ptr to a module
 * 
 * @Note                   - none
 */
 static I2C_RegDef_t* I2C_Get_Module(uint8_t I2C_Module)
 {
     return MapI2CBaseAddress[I2C_Module];
 }

/*********************************************************************************
*                           API functions
*
**********************************************************************************/ 

/********************************************************************************
 * @fn                     - I2C_EnableClk
 *
 * @brief                  - This function enables peripheral clock for the given I2C module
 * 
 * @param[in]              - Run mode clock gating control
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

 void I2C_EnableClk(uint8_t SYSCTL_RCGCI2C_MODULE)
 {
     
    SYSCTL -> RCGCI2C |= SYSCTL_RCGCI2C_MODULE;
 }

  /********************************************************************************
 * @fn                     - I2C_DisableClk
 *
 * @brief                  - This function disables peripheral clock for the given I2C module
 * 
 * @param[in]              - Run mode clock gating control
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void I2C_DisableClk(uint8_t SYSCTL_RCGCI2C_MODULE)
 {
     
    SYSCTL -> RCGCI2C &= ~(SYSCTL_RCGCI2C_MODULE);
 }

   /********************************************************************************
 * @fn                     - I2C_EnableMaster
 *
 * @brief                  - This function enables master function for the I2C module
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_EnableMaster(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    pI2C->I2CMCR |= I2C_MCR_MFE;
}

   /********************************************************************************
 * @fn                     - I2C_DisableMaster
 *
 * @brief                  - This function disables master function for the I2C module
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_DisableMaster(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    pI2C->I2CMCR &= ~(I2C_MCR_MFE);
}

   /********************************************************************************
 * @fn                     - I2C_EnableSlave
 *
 * @brief                  - This function enables slave function for the I2C module
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_EnableSlave(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    // Enable the clock to the slave block
    pI2C->I2CSCSR |= I2C_SCSR_DA;
    
    //Enable slave function
    pI2C->I2CMCR |=I2C_MCR_SFE;   
}

   /********************************************************************************
 * @fn                     - I2C_DisableSlave
 *
 * @brief                  - This function disables slave function for the I2C module
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_DisableSlave(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    //Disable slave function
    pI2C->I2CMCR &= ~(I2C_MCR_SFE);

    //Disable clock to slave block
    pI2->I2CSCSR &= ~(I2C_SCSR_DA);
}

   /********************************************************************************
 * @fn                     - I2C_MasterInit
 *
 * @brief                  - This function initializes the master with desired speed mode
 *                         - High-speed mode should be enabled, before configuring the clock
 * 
 * @param[in]              - I2C module
 * @param[in]              - Desired speed-mode to communicate with slave 
 *                         - Standard, Fast, Fast-Plus and High-speed modes
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_MasterInit(uint8_t I2Cx, uint32_t Speed_Mode uint32_t Clk)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);
    uint32_t TPR;

    // Enable Master function
    I2C_EnableMaster(I2Cx);

    // Compute and set the fasted clock divider which is
    // achieves the fastest speed less than or equal 
    // to the desired speed
    TPR = ((Clk + (2 * 10 * Speed_Mode) - 1) / (2 * 10 Speed_Mode)) - 1;
    pI2C->I2CMTPR = TPR;

    // Check to see if I2C is high speed capable
    if (pI2C->I2CPP & I2C_PP_HS){
        TPR = ((Clk + (2 * 10 * 3400000) - 1) / (2 * 10 3400000)) - 1;
        pI2C->I2CMTPR = I2C_MTPR_HS | TPR;
    }
}

   /********************************************************************************
 * @fn                     - I2C_SlaveInit
 *
 * @brief                  - This function initialize slave with address
 * 
 * @param[in]              - I2C module
 * @param[in]              - I2C slave address
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_SlaveInit(uint8_t I2Cx, uint8_t SlaveAddr)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    //Enable slave function
    I2C_EnableSlave(I2Cx);

    //Set slave address
    pI2C->I2CSOAR = SlaveAddr;
}

   /********************************************************************************
 * @fn                     - I2C_MasterCTRL
 *
 * @brief                  - Controls the state of the master
 * 
 * @param[in]              - I2C module
 * @param[in]              - Different commando which let the user control
 *                         - how master operate
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_MasterCTRL(uint8_t I2Cx, uint8_t CtrlCmd)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    pI2C->I2CMCS = CtrlCmd;
}

   /********************************************************************************
 * @fn                     - I2C_MasterSendData
 *
 * @brief                  - Master sends data
 * 
 * @param[in]              - I2C module
 * @param[in]              - Data to send
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_MasterSendData(uint8_t I2Cx, uint8_t Data)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    pI2C->I2CMDR = Data;
}

   /********************************************************************************
 * @fn                     - I2C_MasterReceiveData
 *
 * @brief                  - Master receive data
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - Data
 * 
 * @Note                   - none
 */
uint8_t I2C_MasterReceiveData(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    return(pI2C->I2CMDR);
}

   /********************************************************************************
 * @fn                     - I2C_SetMasterSlaveAddr
 *
 * @brief                  - This function sets the address the master places on the bus
 * 
 * @param[in]              - I2C module
 * @param[in]              - I2C slave address
 * @param[in]              - Receive or send
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_SetMasterSlaveAddr(uint8_t I2Cx, uint8_t SlaveAddr, uint8_t RS)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);

    pI2C->I2CMSA = (SlaveAddr << 1) | RS; 
}

   /********************************************************************************
 * @fn                     - I2C_SignalLineStatus
 *
 * @brief                  - This function return the status of SDA and SCL
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - Status of SDA(bit 1) and SCL(bit 0)
 * 
 * @Note                   - none
 */
uint8_t I2C_SignalLineStatus(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);
    return(pI2C->I2CMBMON);
}

   /********************************************************************************
 * @fn                     - I2C_MasterBusy
 *
 * @brief                  - Check if the master is busy
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - 1(busy) or 0(not busy)
 * 
 * @Note                   - none
 */
uint8_t I2C_MasterBusy(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);
    return(pI2C->I2CMCS & I2C_MCS_BUSY);
}

   /********************************************************************************
 * @fn                     - I2C_MasterGetErrorStatus
 * 
 * @brief                  - Get the error status on the bus
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - Return the error status
 * 
 * @Note                   - none
 */
uint32_t I2C_MasterGetErrorStatus(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = SSI_Get_Module(SSIx);
    
    //Raw error status
    uint32_t ErrorStatus = pI2C->I2CMCS;

    // If master is busy, there is no error to return
    if(I2C_MasterBusy(I2Cx));
        return(I2C_MCS_NONE_ERROR);
    
    if (ErrorStatus & (I2C_MCS_ERROR | I2C_MCS_ARBLST))
        return(ErrorStatus & (I2C_MCS_ARBLST | I2C_MCS_DATACK | I2C_MCS_ADRACK));
    
    return(I2C_MCS_NONE_ERROR);
}

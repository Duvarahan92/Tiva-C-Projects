#include "tm4c123GH6PM_i2c_driver.h"
#include "tm4c123gh6pm.h"
#include "tm4c123GH6PM_gpio_driver.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/

static I2C_RegDef_t* const MapI2CBaseAddress[4] =
{
    I2C0, 
    I2C1,
    I2C2,
    I2C3
};

static uint32_t const RCGCI2CModule[4] =
{
    SYSCTL_RCGCI2C_R0,
    SYSCTL_RCGCI2C_R1,
    SYSCTL_RCGCI2C_R2,
    SYSCTL_RCGCI2C_R3,
};

static uint32_t const I2CSCLGPIOConfig[4][3] =
{
    {GPIOB_P, PIN_2, GPIO_PCTL_PB2_I2C0SCL},
    {GPIOA_P, PIN_6, GPIO_PCTL_PA6_I2C1SCL},
    {GPIOE_P, PIN_4, GPIO_PCTL_PE4_I2C2SCL},
    {GPIOD_P, PIN_0, GPIO_PCTL_PD0_I2C3SCL},
};

static uint32_t const I2CSDAGPIOConfig[4][3] =
{
    {GPIOB_P, PIN_3, GPIO_PCTL_PB3_I2C0SDA},
    {GPIOA_P, PIN_7, GPIO_PCTL_PA7_I2C1SDA},
    {GPIOE_P, PIN_5, GPIO_PCTL_PE5_I2C2SDA},
    {GPIOD_P, PIN_1, GPIO_PCTL_PD1_I2C3SDA},
};

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

 static void I2C_EnableClk(uint8_t I2Cx)
 {
     
    SYSCTL -> RCGCI2C |= RCGCI2CModule[I2Cx];
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
 static void I2C_DisableClk(uint8_t I2Cx)
 {
     
    SYSCTL -> RCGCI2C &= ~(RCGCI2CModule[I2Cx]);
 }

/********************************************************************************
 * @fn                     - I2C_GPIOTypeSCL
 *
 * @brief                  - This function configure the necessary GPIO pins as SCL to the right I2C module.
 * 
 * @param[in]              - I2C Module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 static void I2C_GPIOTypeSCL(uint8_t I2Cx)
 {
    GPIO_I2CTypeSCL(I2CSCLGPIOConfig[I2Cx][0], I2CSCLGPIOConfig[I2Cx][1], I2CSCLGPIOConfig[I2Cx][2]);
    
 }

 /********************************************************************************
 * @fn                     - I2C_GPIOTypeSDA
 *
 * @brief                  - This function configure the necessary GPIO pins as SDA to the right I2C module.
 * 
 * @param[in]              - I2C Module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 static void I2C_GPIOTypeSDA(uint8_t I2Cx)
 {
    GPIO_I2CTypeSDA(I2CSDAGPIOConfig[I2Cx][0], I2CSDAGPIOConfig[I2Cx][1], I2CSDAGPIOConfig[I2Cx][2]);
    
 }


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
 static I2C_RegDef_t* I2C_Get_Module(uint8_t I2Cx)
 {
     return MapI2CBaseAddress[I2Cx];
 }

    /********************************************************************************
 * @fn                     - I2C_Init
 *
 * @brief                  - This function initialize a I2C module
 * 
 * @param[in]              - I2C module which need to ne initialize
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void I2C_Init(uint8_t I2Cx)
{
    
    I2C_EnableClk(I2Cx); // Enable I2C Clk
    I2C_GPIOTypeSCL(I2Cx); // Set GPIO pin to SCL
    I2C_GPIOTypeSDA(I2Cx); // Sett GPIO pin to SDA
}
 
/********************************************************************************
 * @fn                     - I2C_DeInit
 *
 * @brief                  - This function deinitialize a I2C module
 * 
 * @param[in]              - I2C module which need to ne deinitialize
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_DeInit(uint8_t I2Cx)
{
    I2C_DisableClk(I2Cx);
    // Add reset
}


/*********************************************************************************
*                           API functions
*
**********************************************************************************/ 

/********************************************************************************
 * @fn                     - I2C_EnableLoopBack
 *
 * @brief                  - This function enables loopback
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void I2C_EnableLoopBack(uint8_t I2Cx)
 {
     I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

     pI2C->I2CMCR |= I2C_MCR_LPBK;
 }

 /********************************************************************************
 * @fn                     - I2C_DisableLoopBack
 *
 * @brief                  - This function disables loopback
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void I2C_DisableLoopBack(uint8_t I2Cx)
 {
     I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

     pI2C->I2CMCR &= ~(I2C_MCR_LPBK);
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
void I2C_MasterInit(uint8_t I2Cx, uint32_t Speed_Mode, uint32_t Clk)
{
    I2C_Init(I2Cx);
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);
    uint32_t TPR;

    // Enable Master funtion
    pI2C->I2CMCR |= I2C_MCR_MFE;

    // Compute and set the fasted clock divider which is
    // achieves the fastest speed less than or equal 
    // to the desired speed
    TPR = ((Clk + (2 * 10 * Speed_Mode) - 1) / (2 * 10 * Speed_Mode)) - 1;
    pI2C->I2CMTPR = TPR;

    // Check to see if I2C is high speed capable
    if (pI2C->I2CPP & I2C_PP_HS){
        TPR = ((Clk + (2 * 10 * 3400000) - 1) / (2 * 10 * 3400000)) - 1;
        pI2C->I2CMTPR = I2C_MTPR_HS | TPR;
    }
}

 /********************************************************************************
 * @fn                     - I2C_MasterDeinit
 *
 * @brief                  - This function disables master function for the I2C module
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
static void I2C_MasterDeinit(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

    pI2C->I2CMCR &= ~(I2C_MCR_MFE);
    I2C_DeInit(I2Cx);
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
    I2C_Init(I2Cx);
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);
    
    //Enable slave function
    pI2C->I2CMCR |=I2C_MCR_SFE;

    // Enable the clock to the slave block
    pI2C->I2CSCSR |= I2C_SCSR_DA;

    //Enable slave function
   // pI2C->I2CMCR |=I2C_MCR_SFE; 

    //Set slave address
    pI2C->I2CSOAR = SlaveAddr;
}

/********************************************************************************
 * @fn                     - I2C_SlaveDeinit
 *
 * @brief                  - This function disables slave function for the I2C module
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void I2C_SlaveDeinit(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

    //Disable slave function
    pI2C->I2CMCR &= ~(I2C_MCR_SFE);

    //Disable clock to slave block
    pI2C->I2CSCSR &= ~(I2C_SCSR_DA);

    I2C_DeInit(I2Cx);
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
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

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
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

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
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

    return(pI2C->I2CMDR);
}

/********************************************************************************
 * @fn                     - I2C_SlaveSendData
 *
 * @brief                  - Slave sends data
 * 
 * @param[in]              - I2C module
 * @param[in]              - Data to send
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void I2c_SlaveSendData(uint8_t I2Cx, uint8_t Data)
 {
     I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

     pI2C->I2CSDR = Data;
 }

 /********************************************************************************
 * @fn                     - I2C_SlaveReceiveData
 *
 * @brief                  - Slave receive data
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - Data
 * 
 * @Note                   - none
 */
uint8_t I2C_SlaveReceiveData(uint8_t I2Cx)
{
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);
    return(pI2C->I2CSDR);
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
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);

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
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);
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
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);
    return(pI2C->I2CMCS & I2C_MCS_BUSY);
}

/********************************************************************************
 * @fn                     - I2C_SlaveReceiveBusy
 *
 * @brief                  - Check if the slave is able to receive
 * 
 * @param[in]              - I2C module
 * 
 * @return                 - 1(busy) or 0(not busy)
 * 
 * @Note                   - none
 */
 uint8_t I2C_SlaveReceiveBusy(uint8_t I2Cx)
 {
     I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);
     return(pI2C->I2CSCSR & I2C_SCSR_RREQ);
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
    I2C_RegDef_t *pI2C = I2C_Get_Module(I2Cx);
    
    //Raw error status
    uint32_t ErrorStatus = pI2C->I2CMCS;

    // If master is busy, there is no error to return
    if(I2C_MasterBusy(I2Cx))
        return(I2C_MCS_NONE_ERROR);
    
    if (ErrorStatus & (I2C_MCS_ERROR | I2C_MCS_ARBLST))
        return(ErrorStatus & (I2C_MCS_ARBLST | I2C_MCS_DATACK | I2C_MCS_ADRACK));
    
    return(I2C_MCS_NONE_ERROR);
}

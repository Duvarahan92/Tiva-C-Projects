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
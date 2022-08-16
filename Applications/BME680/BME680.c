#include "BME680.h"
#include "Serial_Interface.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/
#define BME680_CHIP_ID_REG      0xD0         //Chip id address for I2C
#define BME680_CHIP_ID          0x61         //BME680 unique chip id
#define BME680_RESET_REG        0xE0         //Reset address for I2C
#define BME680_RESET            0xB6         //Command to initiate soft reset
#define BME680_MEM_PAGE_REG     0x73         //Register used to get/set spi_mem_page
#define BME680_CTRL_MEAS_REG    0x74         //Register used to get/set osrs_t, osrs_p and mode
#define BME680_CONFIG_REG       0x75         //Register used to get/set filter and spi_3w_en
#define BME680_CTRL_HUM_REG     0x72         //Register used to get/set spi_3w_int_en and osrs_h
#define BME680_CTRL_GAS1_REG    0x71         //Register used to get/set run_gas and nb_conv
#define BME680_CTRL_GAS0_REG    0x70         //Register used to get/set heat_off
#define BME680_GAS_WAIT0_REG    0x6D         //Register used to get/set gas_wait_x, where x is 9 to 0
#define BME680_RES_HEAT0_REG    0x63         //Register used to get/set res_heat_x, where x is 9 to 0
#define BME680_IDAC_HEAT0_REG   0x59         //Register used to get/set idac_heat_x where x is 9 to 0
#define BME680_GAS_R_LSB_REG    0x2B         //Register used to get gas_r<1:0>, gas_valid, heat_stab and gas_range
#define BME680_GAS_R_MSB_REG    0x2A         //Register used to get gas_r<9:2>
#define BME680_HUM_LSB_REG      0x26         //Register used to get hum_lsb
#define BME680_HUM_MSB_REG      0x25         //Register used to get hum_msb
#define BME680_TEMP_XLSB_REG    0x24         //Register used to get temp_xlsb
#define BME680_TEMP_LSB_REG     0x23         //Register used to get temp_lsb
#define BME680_TEMP_MSB_REG     0x22         //Register used to get temp_msb
#define BME680_PRESS_XLSB_REG   0x21         //Register used to get press_xlsb
#define BME680_PRESS_LSB_REG    0x20         //Register used to get press_lsb
#define BME680_PRESS_MSB_REG    0x1F         //Register used to get press_msb 
#define BME680_EAS_STATUS0_REG  0x1D         //Register used to get new_data_0, gas_measuring, measuering and gas_meas_index_0

//*****Data registers*****//

#define BME680_DATA_MSB_MSK     0xFF        //MSB mask for pressure, temp and humidity data <7:0>
#define BME680_DATA_LSB_MSK     0xFF        //LSB mask for pressure, temp and humidity data <7:0>
#define BME680_DATA_XLSB_MSK    0xF0        //XLSB mask for pressure and temp data <7:4>
#define BME680_GAS_MSB_MSK      0xFF        //MSB mask for gas sensor resistance data <7:0>
#define BME680_GAS_LSB_MSK      0xC0        //LSB mask for gas sensor resustance data <7:6>
#define BME680_GAS_RANGE_MSK    0x0F        //Mask for gas sensor resistance range

//*****Status registers*****//

#define BME680_NEW_DATA_MSK     0x80        //New data flag
#define BME680_GAS_MEAS_MSK     0x40        //Gas measuring status flag
#define BME680_MEAS_MSK         0x20        //Measuring status
#define BME680_GAS_VALID_MSK    0x20        //Gas valid bit
#define BME680_HEAT_STAB_MSK    0x10        //Heater stability bit

/*********************************************************************************
*                           API functions
*
**********************************************************************************/ 
/********************************************************************************
 * @fn                     - BME680_Init
 *
 * @brief                  - This function initialize the sesnor.
 * 
 * @param[in]              - Slave address for I2C communication. For SPI 0x00 can be sent. 
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void BME680_Init(uint8_t addr)
{
    Serial_InterfaceInit(addr);
    // GET CALIB DATA
}

/********************************************************************************
 * @fn                     - BME680_DeInit
 *
 * @brief                  - This function deinitialize the sesnor.
 * 
 * @param[in]              - none
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void BME680_DeInit()
{
    BME680_Soft_Reset();
    Serial_InterfaceDeInit();
}

/********************************************************************************
 * @fn                     - BME680_Soft_Reset
 *
 * @brief                  - This function reset the sesnor.
 * 
 * @param[in]              - none
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void BME680_Soft_Reset()
{
    uint8_t reg_addr = BME680_RESET_REG;
    uint8_t cmd = BME680_RESET;
    I2C_Write(&reg_addr, &cmd, 1);
}

#include "BME680.h"
#include "Serial_Interface.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/
#define OS_Len                             2           //Amount of registers containing oversampling data.

#define BME680_CHIP_ID_REG      (uint8_t) 0xD0         //Chip id address for I2C
#define BME680_RESET_REG        (uint8_t) 0xE0         //Reset address for I2C
#define BME680_MEM_PAGE_REG     (uint8_t) 0x73         //Register used to get/set spi_mem_page
#define BME680_CTRL_MEAS_REG    (uint8_t) 0x74         //Register used to get/set osrs_t, osrs_p and mode
#define BME680_CONFIG_REG       (uint8_t) 0x75         //Register used to get/set filter and spi_3w_en
#define BME680_CTRL_HUM_REG     (uint8_t) 0x72         //Register used to get/set spi_3w_int_en and osrs_h
#define BME680_CTRL_GAS1_REG    (uint8_t) 0x71         //Register used to get/set run_gas and nb_conv
#define BME680_CTRL_GAS0_REG    (uint8_t) 0x70         //Register used to get/set heat_off
#define BME680_GAS_WAIT0_REG    (uint8_t) 0x6D         //Register used to get/set gas_wait_x, where x is 9 to 0
#define BME680_RES_HEAT0_REG    (uint8_t) 0x63         //Register used to get/set res_heat_x, where x is 9 to 0
#define BME680_IDAC_HEAT0_REG   (uint8_t) 0x59         //Register used to get/set idac_heat_x where x is 9 to 0
#define BME680_GAS_R_LSB_REG    (uint8_t) 0x2B         //Register used to get gas_r<1:0>, gas_valid, heat_stab and gas_range
#define BME680_GAS_R_MSB_REG    (uint8_t) 0x2A         //Register used to get gas_r<9:2>
#define BME680_HUM_LSB_REG      (uint8_t) 0x26         //Register used to get hum_lsb
#define BME680_HUM_MSB_REG      (uint8_t) 0x25         //Register used to get hum_msb
#define BME680_TEMP_XLSB_REG    (uint8_t) 0x24         //Register used to get temp_xlsb
#define BME680_TEMP_LSB_REG     (uint8_t) 0x23         //Register used to get temp_lsb
#define BME680_TEMP_MSB_REG     (uint8_t) 0x22         //Register used to get temp_msb
#define BME680_PRESS_XLSB_REG   (uint8_t) 0x21         //Register used to get press_xlsb
#define BME680_PRESS_LSB_REG    (uint8_t) 0x20         //Register used to get press_lsb
#define BME680_PRESS_MSB_REG    (uint8_t) 0x1F         //Register used to get press_msb 
#define BME680_EAS_STATUS0_REG  (uint8_t) 0x1D         //Register used to get new_data_0, gas_measuring, measuering and gas_meas_index_0
#define BME680_CHIP_ID          (uint8_t) 0x61         //BME680 unique chip id
#define BME680_RESET            (uint8_t) 0xB6         //Command to initiate soft reset

//*Data*//
#define BME680_DATA_MSB_MSK     (uint8_t) 0xFF         //MSB mask for pressure, temp and humidity data <7:0>
#define BME680_DATA_LSB_MSK     (uint8_t) 0xFF         //LSB mask for pressure, temp and humidity data <7:0>
#define BME680_DATA_XLSB_MSK    (uint8_t) 0xF0         //XLSB mask for pressure and temp data <7:4>
#define BME680_GAS_MSB_MSK      (uint8_t) 0xFF         //MSB mask for gas sensor resistance data <7:0>
#define BME680_GAS_LSB_MSK      (uint8_t) 0xC0         //LSB mask for gas sensor resustance data <7:6>
#define BME680_GAS_RANGE_MSK    (uint8_t) 0x0F         //Mask for gas sensor resistance range

//*Status*//

#define BME680_NEW_DATA_MSK     (uint8_t) 0x80         //New data flag
#define BME680_GAS_MEAS_MSK     (uint8_t) 0x40         //Gas measuring status flag
#define BME680_MEAS_MSK         (uint8_t) 0x20         //Measuring status
#define BME680_GAS_VALID_MSK    (uint8_t) 0x20         //Gas valid bit
#define BME680_HEAT_STAB_MSK    (uint8_t) 0x10         //Heater stability bit

//*General Mask*//
#define BME680_FILTER_MSK       (uint8_t) 0x1C         //Filter mask
#define BME680_OST_MSK          (uint8_t) 0xE0         //Oversampling temperature mask
#define BME680_OSP_MSK          (uint8_t) 0x1C         //Oversampling peressure mask
#define BME680_OSH_MSK          (uint8_t) 0x07         //Oversampling humidity mask
#define BME680_MODE_MSK         (uint8_t) 0x03         //Mode mask

static uint8_t Set_Bits(uint8_t data, uint8_t mask, uint8_t value)
{
    return(((data & ~(mask)) | value));
}

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
    uint8_t reg = BME680_RESET_REG;
    uint8_t cmd = BME680_RESET;
    I2C_Write(&reg, &cmd, 1);
}

/********************************************************************************
 * @fn                     - BME680_Conf_OS
 *
 * @brief                  - This function configure the oversampling to humidity, temperature and pressure
 * 
 * @param[in]              - Oversampling to humidity, temperature and pressure
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void BME680_Conf_OS(uint8_t hum_os, uint8_t temp_os, uint8_t press_os)
{
    uint8_t regs [OS_Len]= {BME680_CTRL_HUM_REG, BME680_CTRL_MEAS_REG};
    uint8_t data[OS_Len]; // mabey 3?

    //Read the previous data from the regs
    I2C_Read(BME680_CTRL_HUM_REG, data, OS_Len);

    data[0]  = Set_Bits(data[0], BME680_OSH_MSK, hum_os);
    data[1]  = Set_Bits(data[1], (BME680_OST_MSK | BME680_OSP_MSK), (temp_os | press_os));

    I2C_Write(regs, data, OS_Len);
}

/********************************************************************************
 * @fn                     - BME680_Conf_Filter
 *
 * @brief                  - This function configure the filter coefficient of the sensor
 * 
 * @param[in]              - Filter coefficent
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void BME680_Conf_Filter(uint8_t filter)
{
    uint8_t reg = BME680_CONFIG_REG;
    uint8_t data;

    //Read the previous data
    I2C_Read(reg, &data, 1);

    data = Set_Bits(data, BME680_FILTER_MSK, filter);

    I2C_Write(&reg, &data, 1);
}

/********************************************************************************
 * @fn                     - BME680_Conf_Mode
 *
 * @brief                  - This function configure the operational mode of the sensor
 * 
 * @param[in]              - mode
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void BME680_Conf_Mode(uint8_t mode)
{
    uint8_t reg = BME680_CTRL_MEAS_REG;
    uint8_t data;

    I2C_Read(reg, &data, 1);

   data = Set_Bits(data, BME680_MODE_MSK, mode);

   I2C_Write(&reg, &data, 1); 

}

/********************************************************************************
 * @fn                     - BME680_Conf_Heater
 *
 * @brief                  - This function configure the gas and heater of the sensor
 * 
 * @param[in]              - heat time duration
 * @param[in]              - target temperature
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void BME680_Conf_Heater(uint8_t gas_wait, uint8_t heat_temp)
{
    uint8_t gas_wait_reg = BME680_GAS_WAIT0_REG;
    uint8_t res_heat_reg = BME680_RES_HEAT0_REG; 
    uint8_t gas_ctrl_regs[2] = {BME680_CTRL_GAS0_REG, BME680_CTRL_GAS1_REG};
    uint8_t gas_ctrl_data[2];
    
    BME680_Conf_Mode(BME680_SLEEP_MODE);
}
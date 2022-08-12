/*BME680.h*/
#include <stdint.h>

#ifndef INC_BME680_H_
#define INC_BME680_H_

//*****Can be used inside source file*****//

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


// *****Can be used inside header file*****//

#define BME680_I2C_LOW_ADDR     0x76        //Low I2C address is set by connecting SDO to GND
#define BME680_I2C_HIGH_ADDR    0x77        //High I2C address is set by connecting SDO to Vddio

//*****General control registers*****//

#define BME680_SPI_MEM_PAGE1    0x10        //SPI memory page 1
#define BME680_SPI_MEM_PAGE0    0x00        //SPI memory page 0
#define BME680_SPI_3W_EN        0x01        //Enable SPI 3 wire mode
#define BME680_SPI_3W_INT_EN    0x40        //Enbable SPI 3 wire interrupt

//*****Temperature, pressure and relative humidty control registers*****//

#define BME680_FILTER_0         0x00        //Filter coeffcient 0
#define BME680_FILTER_1         0x04        //Filter coeffcient 1
#define BME680_FILTER_3         0x08        //Filter coeffcient 3
#define BME680_FILTER_7         0x0C        //Filter coeffcient 7
#define BME680_FILTER_15        0x10        //Filter coeffcient 15
#define BME680_FILTER_31        0x14        //Filter coeffcient 31
#define BME680_FILTER_63        0x18        //Filter coeffcient 63
#define BME680_FILTER_127       0x1C        //Filter coeffcient 127

#define BME680_OSH_0X           0x00        //Skip measurement of humidity sensor
#define BME680_OSH_1X           0x01        //1 measurement of humidity sensor
#define BME680_OSH_2X           0x02        //2 measurements of humidity sensor
#define BME680_OSH_4X           0x03        //4 measurements of humidity sensor
#define BME680_OSH_8X           0x04        //8 measurements of humidity sensor
#define BME680_OSH_16X          0x05        //16 measurements of humidity sensor

#define BME680_OST_0X           0x00        //Skip measurement of temperature data
#define BME680_OST_1X           0x20        //1 measurement of temperature data
#define BME680_OST_2X           0x40        //2 measurement of temperature data
#define BME680_OST_4X           0x60        //4 measurement of temperature data
#define BME680_OST_8X           0x80        //8 measurement of temperature data
#define BME680_OST_16X          0xA0        //16 measurement of temperature data

#define BME680_OSP_0X           0x00        //Skip measurement of pressure data
#define BME680_OSP_1X           0x04        //1 measurement of pressure data
#define BME680_OSP_2X           0x08        //2 measurement of pressure data
#define BME680_OSP_4X           0x0C        //4 measurement of pressure data
#define BME680_OSP_8X           0x10        //8 measurement of pressure data
#define BME680_OSP_16X          0x14        //16 measurement of pressure data

#define BME680_SLEEP_MODE       0x00        //Put the sensor in sleep mode
#define BME680_FORCE_MODE       0x01        //Put the sensor in force mode

//*****Gas control registers*****//

#define BME680_HEAT_OFF         0x08        //Turn off current injected to heater
#define BME680_HEAT_ON          0x00        //Turn on current injected to heater

#define BME680_NBC_0            0x00        //Heater profile set-point index 0
#define BME680_NBC_1            0x01        //Heater profile set-point index 1
#define BME680_NBC_2            0x02        //Heater profile set-point index 2
#define BME680_NBC_3            0x03        //Heater profile set-point index 3
#define BME680_NBC_4            0x04        //Heater profile set-point index 4
#define BME680_NBC_5            0x05        //Heater profile set-point index 5
#define BME680_NBC_6            0x06        //Heater profile set-point index 6
#define BME680_NBC_7            0x07        //Heater profile set-point index 7
#define BME680_NBC_8            0x08        //Heater profile set-point index 8
#define BME680_NBC_9            0x09        //Heater profile set-point index 9

#define BME680_RUN_GAS          0x10        //Start the gas conversions
#define BME680_STOP_GAS         0x00        //Stop the gas conversions


#endif  // __BME680_H__
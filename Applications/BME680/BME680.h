/*BME680.h*/
#include <stdint.h>

#ifndef INC_BME680_H_
#define INC_BME680_H_

#define BME680_I2C_LOW_ADDR     (uint8_t) 0x76        //Low I2C address is set by connecting SDO to GND
#define BME680_I2C_HIGH_ADDR    (uint8_t) 0x77        //High I2C address is set by connecting SDO to Vddio

//*****General control registers*****//

#define BME680_SPI_MEM_PAGE1    (uint8_t) 0x10        //SPI memory page 1
#define BME680_SPI_MEM_PAGE0    (uint8_t) 0x00        //SPI memory page 0
#define BME680_SPI_3W_EN        (uint8_t) 0x01        //Enable SPI 3 wire mode
#define BME680_SPI_3W_INT_EN    (uint8_t) 0x40        //Enbable SPI 3 wire interrupt

//*****Temperature, pressure and relative humidty control registers*****//

#define BME680_FILTER_0         (uint8_t) 0x00        //Filter coeffcient 0
#define BME680_FILTER_1         (uint8_t) 0x04        //Filter coeffcient 1
#define BME680_FILTER_3         (uint8_t) 0x08        //Filter coeffcient 3
#define BME680_FILTER_7         (uint8_t) 0x0C        //Filter coeffcient 7
#define BME680_FILTER_15        (uint8_t) 0x10        //Filter coeffcient 15
#define BME680_FILTER_31        (uint8_t) 0x14        //Filter coeffcient 31
#define BME680_FILTER_63        (uint8_t) 0x18        //Filter coeffcient 63
#define BME680_FILTER_127       (uint8_t) 0x1C        //Filter coeffcient 127

//Used in BME680_Conf_OS function
#define BME680_OSH_0X           (uint8_t) 0x00        //Skip measurement of humidity sensor
#define BME680_OSH_1X           (uint8_t) 0x01        //1 measurement of humidity sensor
#define BME680_OSH_2X           (uint8_t) 0x02        //2 measurements of humidity sensor
#define BME680_OSH_4X           (uint8_t) 0x03        //4 measurements of humidity sensor
#define BME680_OSH_8X           (uint8_t) 0x04        //8 measurements of humidity sensor
#define BME680_OSH_16X          (uint8_t) 0x05        //16 measurements of humidity sensor

#define BME680_OST_0X           (uint8_t) 0x00        //Skip measurement of temperature data
#define BME680_OST_1X           (uint8_t) 0x20        //1 measurement of temperature data
#define BME680_OST_2X           (uint8_t) 0x40        //2 measurement of temperature data
#define BME680_OST_4X           (uint8_t) 0x60        //4 measurement of temperature data
#define BME680_OST_8X           (uint8_t) 0x80        //8 measurement of temperature data
#define BME680_OST_16X          (uint8_t) 0xA0        //16 measurement of temperature data

#define BME680_OSP_0X           (uint8_t) 0x00        //Skip measurement of pressure data
#define BME680_OSP_1X           (uint8_t) 0x04        //1 measurement of pressure data
#define BME680_OSP_2X           (uint8_t) 0x08        //2 measurement of pressure data
#define BME680_OSP_4X           (uint8_t) 0x0C        //4 measurement of pressure data
#define BME680_OSP_8X           (uint8_t) 0x10        //8 measurement of pressure data
#define BME680_OSP_16X          (uint8_t) 0x14        //16 measurement of pressure data

#define BME680_SLEEP_MODE       (uint8_t) 0x00        //Put the sensor in sleep mode
#define BME680_FORCE_MODE       (uint8_t) 0x01        //Put the sensor in force mode

//*****Gas control registers*****//

#define BME680_HEAT_OFF         (uint8_t) 0x08        //Turn off current injected to heater
#define BME680_HEAT_ON          (uint8_t) 0x00        //Turn on current injected to heater

#define BME680_NBC_0            (uint8_t) 0x00        //Heater profile set-point index 0
#define BME680_NBC_1            (uint8_t) 0x01        //Heater profile set-point index 1
#define BME680_NBC_2            (uint8_t) 0x02        //Heater profile set-point index 2
#define BME680_NBC_3            (uint8_t) 0x03        //Heater profile set-point index 3
#define BME680_NBC_4            (uint8_t) 0x04        //Heater profile set-point index 4
#define BME680_NBC_5            (uint8_t) 0x05        //Heater profile set-point index 5
#define BME680_NBC_6            (uint8_t) 0x06        //Heater profile set-point index 6
#define BME680_NBC_7            (uint8_t) 0x07        //Heater profile set-point index 7
#define BME680_NBC_8            (uint8_t) 0x08        //Heater profile set-point index 8
#define BME680_NBC_9            (uint8_t) 0x09        //Heater profile set-point index 9

#define BME680_RUN_GAS          (uint8_t) 0x10        //Start the gas conversions
#define BME680_STOP_GAS         (uint8_t) 0x00        //Stop the gas conversions

/*********************************************************************************
*                        API supported by this driver
*
**********************************************************************************/

/*
 * Sensor setup
 *
 */
void BME680_Init(uint8_t addr);
void BME680_DeInit();
void BME680_Soft_Reset();

/*
 * Configure sesnor
 *
 */
void BME680_Conf_OS(uint8_t hum_os, uint8_t temp_os, uint8_t press_os);
void BME680_Conf_Filter(uint8_t filter);
void BME680_Conf_Heater(uint8_t gas_wait, uint8_t heat_temp);
void BME680_Conf_Mode(uint8_t mode);

#endif  // __BME680_H__
/*BME680.h*/
#include <stdint.h>

#ifndef INC_BME680_H_
#define INC_BME680_H_

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

/*********************************************************************************
*                        API supported by this driver
*
**********************************************************************************/

/*
 * Sensor setup
 *
 */
void BME689_Init(uint8_t addr);
void BME680_DeInit();
void BME680_Soft_Reset();

#endif  // __BME680_H__
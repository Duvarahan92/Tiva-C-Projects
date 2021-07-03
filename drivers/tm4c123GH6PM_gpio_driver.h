/* tm4c123GH6PM_gpio_driver.h */

#ifndef INC_TM4C123GH6PM_GPIO_DRIVER_H_
#define INC_TM4C123GH6PM_GPIO_DRIVER_H_

#include "tm4c123gh6pm.h"

/*
/ This is a Confiuration structure for a GPIO pin
*/
typedef struct
{
  uint8_t GPIO_PinDir;                  /*!< GPIO Data Direction >*/
  uint8_t GPIO_AFSEL;                   /*!< GPIO Alternate Function Select >*/
  uint8_t GPIO_DR2R;                    /*!< Output Pad 2-mA Drive Enable >*/
  uint8_t GPIO_DR4R;                    /*!< Output Pad 4-mA Drive Enable >*/
  uint8_t GPIO_DR8R;                    /*!< Output Pad 8-mA Drive Enable >*/
  uint8_t GPIO_OODR;                    /*!< Output Pad Open Drain Enable >*/
  uint8_t GPIO_PUR;                     /*!< Pad Weak Pull-Up Enable >*/
  uint8_t GPIO_PDR;                     /*!< Pad Weak Pull-Down Enable >*/
  uint8_t GPIO_SLR;                     /*!< Slew Rate Limit Enable (8-mA drive only) >*/
  uint8_t GPIO_DEN;                     /*!< Digital Enable >*/
  uint8_t GPIO_LOCK;                    /*!< GPIO Lock >*/
  uint8_t GPIO_CR;                      /*!< GPIO Commit >*/
  uint8_t GPIO_AMSEL;                   /*!< GPIO Analog Mode Select >*/
  uint32_t GPIO_PCTL;                   /*!< GPIO port control >*/
  uint8_t GPIO_ADCCTL;                  /*!< ADC Trigger Enable >*/
  uint8_t GPIO_DMACTL;                  /*!< μDMA Trigger Enable >*/

  //uint8_t GPIO_PinDir;                  /*!< GPIO Data Direction >*/
  
  

}GPIO_PinConfig_t;

/*
/ This is a handle structure for GPIO pin
*/

typedef struct
{

  GPIO_RegDef_t *GPIOx;                  /*!< Pointer to the base address >*/
  GPIO_PinConfig_t GPIO_PinConfig;       /*!< Struct which contains pin configuration settings >*/

}GPIO_Handle_t;  







#endif // __TM4C123GH6PM_GPIO_DRIVER_H__

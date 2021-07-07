
//#include "tm4c123gh6pm.h"
#include "tm4c123GH6PM_gpio_driver.h"

/*
 *Internal Protype functions
 *
 */

uint8_t GPIO_Check_Port(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_Check_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

  
 


/********************************************************************************
 * @fn                     - GPIO_CLK_CTRL
 *
 * @brief                  - This function enbales or disables peripheral clock for the given GPIO port
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
uint8_t GPIO_CLK_CTRL(GPIO_RegDef_t *pGPIOx, uint8_t Ctrl)
{
    if (!GPIO_Check_Port(pGPIOx))
        return FALSE;
    if (Ctrl == ENABLE)
    {
        if (pGPIOx == GPIOA || pGPIOx == GPIOA_AHB)
        {
            SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_R0;
            return TRUE;
        }else if (pGPIOx == GPIOB || pGPIOx == GPIOB_AHB)
        {
            SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_R1;
            return TRUE;
        }else if (pGPIOx == GPIOC || pGPIOx == GPIOC_AHB)
        {
            SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_R2;
            return TRUE;
        }else if (pGPIOx == GPIOD || pGPIOx == GPIOD_AHB)
        {
            SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_R3;
            return TRUE;
        }else if (pGPIOx == GPIOE || pGPIOx == GPIOE_AHB)
        {
            SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_R4;
            return TRUE;
        }else if (pGPIOx == GPIOF || pGPIOx == GPIOF_AHB)
        {
            SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_R5;
            return TRUE;
        }

    }else if (Ctrl == DISABLE)
    {
        if (pGPIOx == GPIOA || pGPIOx == GPIOA_AHB)
        {
            SYSCTL -> RCGCGPIO &= ~(SYSCTL_RCGCGPIO_R0);
            return TRUE;
        }else if (pGPIOx == GPIOB || pGPIOx == GPIOB_AHB)
        {
            SYSCTL -> RCGCGPIO &= ~(SYSCTL_RCGCGPIO_R1);
            return TRUE;
        }else if (pGPIOx == GPIOC || pGPIOx == GPIOC_AHB)
        {
            SYSCTL -> RCGCGPIO &= ~(SYSCTL_RCGCGPIO_R2);
            return TRUE;
        }else if (pGPIOx == GPIOD || pGPIOx == GPIOD_AHB)
        {
            SYSCTL -> RCGCGPIO &=  ~(SYSCTL_RCGCGPIO_R3);
            return TRUE;
        }else if (pGPIOx == GPIOE || pGPIOx == GPIOE_AHB)
        {
            SYSCTL -> RCGCGPIO &= ~(SYSCTL_RCGCGPIO_R4);
            return TRUE;
        }else if (pGPIOx == GPIOF || pGPIOx == GPIOF_AHB)
        {
            SYSCTL -> RCGCGPIO &= ~(SYSCTL_RCGCGPIO_R5);
            return TRUE;
        }

    }
    
    return FALSE;
}

/********************************************************************************
 * @fn                     - GPIO_EnableBus
 *
 * @brief                  - This function enables bus AHB or APB
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - Bus to enable macros
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */

uint8_t GPIO_EnableBus(GPIO_RegDef_t *pGPIOx)
{
    
    if (!GPIO_Check_Port(pGPIOx))
        return FALSE;
    
    switch ((uint32_t) pGPIOx)
    {
    case (uint32_t) GPIOA:
        SYSCTL -> GPIOHBCTL &= ~(SYSCTL_GPIOHBCTL_PORTA);
        return TRUE;
    case (uint32_t) GPIOB:
        SYSCTL -> GPIOHBCTL &= ~(SYSCTL_GPIOHBCTL_PORTB);
        return TRUE;
    case (uint32_t) GPIOC:
        SYSCTL -> GPIOHBCTL &= ~(SYSCTL_GPIOHBCTL_PORTC);
        return TRUE;
    case (uint32_t) GPIOD:
        SYSCTL -> GPIOHBCTL &= ~(SYSCTL_GPIOHBCTL_PORTD);
        return TRUE;
    case (uint32_t) GPIOE:
        SYSCTL -> GPIOHBCTL &= ~(SYSCTL_GPIOHBCTL_PORTE);
        return TRUE;
    case (uint32_t) GPIOF:
        SYSCTL -> GPIOHBCTL &= ~(SYSCTL_GPIOHBCTL_PORTF);
        return TRUE;
    case (uint32_t) GPIOA_AHB:
        SYSCTL -> GPIOHBCTL |= SYSCTL_GPIOHBCTL_PORTA;
        return TRUE;
    case (uint32_t) GPIOB_AHB:
        SYSCTL -> GPIOHBCTL |= SYSCTL_GPIOHBCTL_PORTB;
        return TRUE;
    case (uint32_t) GPIOC_AHB:
        SYSCTL -> GPIOHBCTL |= SYSCTL_GPIOHBCTL_PORTC;
        return TRUE;
    case (uint32_t) GPIOD_AHB:
        SYSCTL -> GPIOHBCTL |= SYSCTL_GPIOHBCTL_PORTD;
        return TRUE;
    case (uint32_t) GPIOE_AHB:
        SYSCTL -> GPIOHBCTL |= SYSCTL_GPIOHBCTL_PORTE;
        return TRUE;
    case (uint32_t) GPIOF_AHB:
        SYSCTL -> GPIOHBCTL |= SYSCTL_GPIOHBCTL_PORTF;
        return TRUE;
    }
    return FALSE;
}

/********************************************************************************
 * @fn                     - GPIO_Init
 *
 * @brief                  - This function initialize GPIO pin
 * 
 * @param[in]              - handle structure for a GPIO pin.
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
uint8_t GPIO_Init(GPIO_Handle_t *pGPIOHandle){
    
    uint32_t temp;

    //Check if port and pin number is valid
    if(!GPIO_Check_Pin(pGPIOHandle->GPIOx, pGPIOHandle->GPIO_PinConfig.GPIO_PinDir))
        return FALSE; 

    //1. Configure GPIO direction
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinDir << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODIR |= temp;

    //2. Confgure Alternate function
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_AFSEL << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOAFSEL |= temp;

    //3. Confgure 2mA drive
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DR2R << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODR2R |= temp;

    //4. Confgure 4mA drive
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DR4R << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODR4R |= temp;

    //5. Confgure 8mA drive
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DR8R << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODR8R |= temp;

    //6. Confgure open drain
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_OODR << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOODR |= temp;

    //7. Confgure pull up
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PUR << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOPUR |= temp;

    //8. Confgure pull down
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PDR << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOPDR |= temp;

    //9. Confgure slew rate
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_SLR << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOSLR |= temp;

    //10. Confgure digital function
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DEN << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODEN |= temp;

    //11. Confgure Lock
    pGPIOHandle -> GPIOx -> GPIOLOCK |= pGPIOHandle -> GPIO_PinConfig.GPIO_LOCK;

    //12. Confgure commit
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_CR << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOCR |= temp;

    //13. Confgure analog funtion
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_AMSEL << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOAMSEL |= temp;

    //14. Confgure port control
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PCTL << (4 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOPCTL |= temp;

    //15. Confgure adc trigger
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_ADCCTL << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOADCCTL |= temp;

    //16. Confgure dma trigger
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DMACTL << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODMACTL |= temp;

    return TRUE;
}

/********************************************************************************
 * @fn                     - GPIO_DeInit
 *
 * @brief                  - This function reset the GPIO peripheral
 * 
 * @param[in]              - base address of the gpio peripheral
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
uint8_t GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

    if (!GPIO_Check_Port(pGPIOx))
        return FALSE;

    if (pGPIOx == GPIOA || pGPIOx == GPIOA_AHB)
        {
            SYSCTL -> SRGPIO |= SYSCTL_SRGPIO_R0;
            SYSCTL -> SRGPIO &= ~(SYSCTL_SRGPIO_R0);
            return TRUE;
        }else if (pGPIOx == GPIOB || pGPIOx == GPIOB_AHB)
        {
            SYSCTL -> SRGPIO |= SYSCTL_SRGPIO_R1;
            SYSCTL -> SRGPIO &= ~(SYSCTL_SRGPIO_R1);
            return TRUE;
        }else if (pGPIOx == GPIOC || pGPIOx == GPIOC_AHB)
        {
            SYSCTL -> SRGPIO |= SYSCTL_SRGPIO_R2;
            SYSCTL -> SRGPIO &= ~(SYSCTL_SRGPIO_R2);
            return TRUE;
        }else if (pGPIOx == GPIOD || pGPIOx == GPIOD_AHB)
        {
            SYSCTL -> SRGPIO |= SYSCTL_SRGPIO_R3;
            SYSCTL -> SRGPIO &= ~(SYSCTL_SRGPIO_R3);
            return TRUE;
        }else if (pGPIOx == GPIOE || pGPIOx == GPIOE_AHB)
        {
            SYSCTL -> SRGPIO |= SYSCTL_SRGPIO_R4;
            SYSCTL -> SRGPIO &= ~(SYSCTL_SRGPIO_R4);
            return TRUE;
        }else if (pGPIOx == GPIOF || pGPIOx == GPIOF_AHB)
        {
            SYSCTL -> SRGPIO |= SYSCTL_SRGPIO_R5;
            SYSCTL -> SRGPIO &= ~(SYSCTL_SRGPIO_R5);
            return TRUE;
        }
    
    return FALSE;

}

/********************************************************************************
 * @fn                     - GPIO_ReadInputPort
 *
 * @brief                  - This function reads the GPIO port
 * 
 * @param[in]              - base address of the gpio port
 * 
 * @return                 - reads values from 0 to 255 in binary. Return -1 if non valid port
 * 
 * @Note                   - none
 */

uint8_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx) 
{
    //Check if port is valid
    if(!GPIO_Check_Port(pGPIOx))
        return -1;
    
    return pGPIOx -> GPIODATA;
}

/********************************************************************************
 * @fn                     - GPIO_ReadInputPin
 *
 * @brief                  - This function reads from GPIO pin
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - GPIO pin number
 * 
 * @return                 - reads 1 or 0, if port and/or pin is nor valid return -1
 * 
 * @Note                   - none
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    //Check if port and pin number is valid
    if(!GPIO_Check_Pin(pGPIOx, PinNumber))
        return -1;
    
    // Address bit associated with the data bit
    uint8_t address = 1 << PinNumber;
    
    return pGPIOx->GPIODATA_BITS[address];
}

/********************************************************************************
 * @fn                     - GPIO_WriteOutputPort
 *
 * @brief                  - This function write to GPIO port
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - write value from 0 to 255
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
uint8_t GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value){

    //Check if Value is valid
    if (!(Value >= 0 && Value <= 255))
        return FALSE;
    
    //Check if port is valid
    if (!GPIO_Check_Port(pGPIOx))
        return FALSE;

    pGPIOx-> GPIODATA = Value;
    return TRUE;
}

/********************************************************************************
 * @fn                     - GPIO_WriteOutputPin
 *
 * @brief                  - This function wrtie to GPIO pin
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - GPIO pin
 * @param[in]              - 0 or 1
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */

uint8_t GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {

    //Check if Value is valid
    if (!(Value >= 0 && Value <= 255))
        return FALSE;
    
    //Check if pin is valid
    if (!GPIO_Check_Pin(pGPIOx, PinNumber))
        return FALSE;

    //address bit associated with that data
    uint8_t address = 1 << PinNumber;
    uint8_t data = Value << PinNumber;

    pGPIOx -> GPIODATA_BITS[address] = data;
    return TRUE;
}

/********************************************************************************
 * @fn                     - GPIO_ToggleOutputPin
 *
 * @brief                  - This function toogles GPIO pin
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - GPIO pin number
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
uint8_t GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    //Check if port and pin is valid
    if(!GPIO_Check_Pin(pGPIOx, PinNumber))
        return FALSE;

    pGPIOx->GPIODATA ^= 1 << PinNumber;
    return TRUE;
}

/********************************************************************************
 * @fn                     - GPIO_IRQConfig
 *
 * @brief                  - This function enbales or disables peripheral clock for the given GPIO port
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
void GPIO_IRQConfig(uint8_t IRQn, uint8_t IRQPriority, uint8_t ctrl);

/********************************************************************************
 * @fn                     - GPIO_IRQHandling
 *
 * @brief                  - This function enbales or disables peripheral clock for the given GPIO port
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
void GPIO_IRQHandling(uint8_t PinNumber);

/*********************************************************************************
*                        Internal functions
*
**********************************************************************************/

/********************************************************************************
 * @fn                     - GPIO_Check_Port
 *
 * @brief                  - This function check if GPIO port is valid
 * 
 * @param[in]              - base address of the gpio peripheral
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
uint8_t GPIO_Check_Port(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA || pGPIOx == GPIOB || pGPIOx == GPIOC || pGPIOx == GPIOD || pGPIOx == GPIOE || pGPIOx == GPIOF ||
        pGPIOx == GPIOA_AHB || pGPIOx == GPIOB_AHB || pGPIOx == GPIOC_AHB || pGPIOx == GPIOD_AHB || pGPIOx == GPIOE_AHB || pGPIOx == GPIOF_AHB)
        return TRUE;
    else
        return FALSE;
}

/********************************************************************************
 * @fn                     - GPIO_Check_Pin
 *
 * @brief                  - This function check if pin number is valid
 * 
 * @param[in]              - base address of the GPIO port
 * @param[in]              - pin number of the GPIO port
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
uint8_t GPIO_Check_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t valid = GPIO_Check_Port(pGPIOx);

    if (pGPIOx == GPIOA ||pGPIOx == GPIOB ||pGPIOx == GPIOC || pGPIOx == GPIOD ||
        pGPIOx == GPIOA_AHB || pGPIOx == GPIOB_AHB || pGPIOx == GPIOC_AHB || pGPIOx == GPIOD_AHB)
    {
        if (PinNumber >= 0 && PinNumber <= 7)
            valid &= TRUE;
        else
            valid &= FALSE;
    }else if (pGPIOx == GPIOE ||pGPIOx == GPIOE_AHB)
    {
        if (PinNumber >= 0 && PinNumber <= 5)
            valid &= TRUE;
        else
            valid &= FALSE;
    }else if (pGPIOx == GPIOF ||pGPIOx == GPIOF_AHB)
    {
        if (PinNumber >= 0 && PinNumber <= 4)
            valid &= TRUE;
        else
            valid &= FALSE;
    }
    return valid;
}


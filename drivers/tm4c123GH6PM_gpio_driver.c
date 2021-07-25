#include "tm4c123GH6PM_gpio_driver.h"
#include "tm4c123gh6pm.h"

/*********************************************************************************
*                        Internal functions and variables
*
**********************************************************************************/

static GPIO_RegDef_t* MapGPIOBaseAddress[12] =
{
    GPIOA, GPIOA_AHB,
    GPIOB, GPIOB_AHB,
    GPIOC, GPIOC_AHB,
    GPIOD, GPIOD_AHB,
    GPIOE, GPIOE_AHB,
    GPIOF, GPIOF_AHB
};

/********************************************************************************
 * @fn                     - GPIO_Get_Port
 *
 * @brief                  - This function gets the struct ptr to a port
 * 
 * @param[in]              - GPIO port
 * 
 * @return                 - Struct ptr to a port
 * 
 * @Note                   - none
 */
 static GPIO_RegDef_t* GPIO_Get_Port(uint8_t GPIO_Port)
 {
     return MapGPIOBaseAddress[GPIO_Port];
 }

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
static uint8_t GPIO_Check_Port(uint8_t GPIOPort)
{
    for (int i = GPIOA_P; i <= GPIOF_AHB_P; i++)
    {
        if (i == GPIOPort)
            return TRUE;
    }

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
static uint8_t GPIO_Check_Pin(uint8_t GPIOx, uint8_t PinNumber)
{
    uint8_t valid = GPIO_Check_Port(GPIOx);
    GPIO_RegDef_t *pGPIOx = GPIO_Get_Port(GPIOx);

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

/*********************************************************************************
*                           API functions
*
**********************************************************************************/ 

/********************************************************************************
 * @fn                     - GPIO_EnableClk
 *
 * @brief                  - This function enbales peripheral clock for the given GPIO port
 * 
 * @param[in]              - GPIO port to enable
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void GPIO_EnableClk(uint8_t SYSCTL_RCGCGPIO_MODULE)
{
    SYSCTL -> RCGCGPIO |= SYSCTL_RCGCGPIO_MODULE;        
}

/********************************************************************************
 * @fn                     - GPIO_DisableClk
 *
 * @brief                  - This function disbales peripheral clock for the given GPIO port
 * 
 * @param[in]              - GPIO port to disable clk
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void GPIO_DisableClk(uint8_t SYSCTL_RCGCGPIO_Portx)
 {
     SYSCTL -> RCGCGPIO &= ~(SYSCTL_RCGCGPIO_Portx);
 }

/********************************************************************************
 * @fn                     - GPIO_SelectBus
 *
 * @brief                  - This function selects between bus AHB or APB
 * 
 * @param[in]              - GPIO port
 * @param[in]              - AHB or APB macros
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void GPIO_SelectBus(uint8_t SYSCTL_GPIOHBCTL_PORTx, uint8_t Bus)
{
    if (Bus == AHB)
        SYSCTL -> GPIOHBCTL |= Bus;
     SYSCTL -> GPIOHBCTL &= ~(Bus);
  
}

/********************************************************************************
 * @fn                     - GPIO_ModeSet
 *
 * @brief                  - This function set mode for the GPIO pin(s)
 * 
 * @param[in]              -  GPIO port
 * @param[in]              -  GPIO pin(s)
 * @param[in]              -  GPIO mode
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void GPIO_ModeSet(uint8_t GPIOx, uint8_t Pinx, uint8_t PinIO)
 {
     // Get pointer to GPIO port
     GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

     // Set the pin mode
    pGPIO -> GPIODIR = ((PinIO & 1) ? (pGPIO -> GPIODIR | Pinx) : (pGPIO -> GPIODIR & ~(Pinx)));
    pGPIO -> GPIOAFSEL = ((PinIO & 2) ? (pGPIO -> GPIOAFSEL | Pinx) : (pGPIO -> GPIOAFSEL & ~(Pinx)));
    pGPIO -> GPIOADCCTL = ((PinIO & 3) ? (pGPIO -> GPIOADCCTL | Pinx) : (pGPIO -> GPIOADCCTL & ~(Pinx)));
    pGPIO -> GPIODMACTL = ((PinIO & 4) ? (pGPIO -> GPIODMACTL | Pinx) : (pGPIO -> GPIODMACTL & ~(Pinx)));
 }

 /********************************************************************************
 * @fn                     - GPIO_PadConfig
 *
 * @brief                  - This function config GPIO pad(s)
 * 
 * @param[in]              -  GPIO port
 * @param[in]              -  GPIO pin(s)
 * @param[in]              -  GPIO drive strength
 * @param[in]              -  GPIO pin type
 * 
 * @return                 - none
 * 
 * @Note                   - none 
 */
 void GPIO_PadConfig(uint8_t GPIOx, uint8_t Pinx, uint8_t Strength, uint8_t PinType)
 {
     // Get pointer to GPIO port
     GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

      // Set drive strength
      pGPIO -> GPIODR2R = ((Strength & 1) ? (pGPIO -> GPIODR2R | Pinx) : (pGPIO -> GPIODR2R & ~(Pinx)));
      pGPIO -> GPIODR4R = ((Strength & 2) ? (pGPIO -> GPIODR4R | Pinx) : (pGPIO -> GPIODR4R & ~(Pinx)));
      pGPIO -> GPIODR8R = ((Strength & 3) ? (pGPIO -> GPIODR8R | Pinx) : (pGPIO -> GPIODR8R & ~(Pinx)));

      // Set pin type
      pGPIO -> GPIOODR = ((PinType & 1) ? (pGPIO -> GPIOODR | Pinx) : (pGPIO -> GPIOODR & ~(Pinx)));
      pGPIO -> GPIOPUR = ((PinType & 2) ? (pGPIO -> GPIOPUR| Pinx) : (pGPIO -> GPIOPUR & ~(Pinx)));
      pGPIO -> GPIOPDR = ((PinType & 4) ? (pGPIO -> GPIOPDR | Pinx) : (pGPIO -> GPIOPDR & ~(Pinx)));
      pGPIO -> GPIODEN = ((PinType & 8) ? (pGPIO -> GPIODEN | Pinx) : (pGPIO -> GPIODEN & ~(Pinx)));
      pGPIO -> GPIOAMSEL = ((PinType == 0) ? (pGPIO -> GPIOAMSEL | Pinx) : (pGPIO -> GPIOAMSEL & ~(Pinx)));
}

/********************************************************************************
 * @fn                     - GPIO_PinConfig
 *
 * @brief                  - This function config GPIO pin(s)
 * 
 * @param[in]              - GPIO port
 * @param[in]              - Peripheral signal for a GPIO pin
 * 
 * @return                 - none
 * 
 * @Note                   - Remember to set pin to alternate function with GPIO_ModeSet function
 */
void GPIO_PinConfig(uint8_t GPIOx, uint32_t GPIO_PCTL)
{
    // Get pointer to GPIO port
     GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

     pGPIO -> GPIOPCTL |= GPIO_PCTL;
}

/********************************************************************************
 * @fn                     - GPIO_EnableSLR
 *
 * @brief                  - This function enables slew rate control on the pin
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin
 * 
 * @return                 - none
 * 
 * @Note                   - 8-mA drive only
 */
 void GPIO_EnableSLR(uint8_t GPIOx, uint8_t Pinx)
 {
      GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
      
      pGPIO -> GPIOSLR |= Pinx;
 }

 /********************************************************************************
 * @fn                     - GPIO_DisableSLR
 *
 * @brief                  - This function disables slew rate control on the pin
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin
 * 
 * @return                 - none
 * 
 * @Note                   - 8-mA drive only
 */
 void GPIO_DisableSLR(uint8_t GPIOx, uint8_t Pinx)
 {
     GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
      
    pGPIO -> GPIOSLR &= ~(Pinx);
 }

/********************************************************************************
 * @fn                     - GPIO_Reset
 *
 * @brief                  - This function reset the GPIO peripheral
 * 
 * @param[in]              - GPIO port
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void GPIO_Reset(uint8_t SYSCTL_SRGPIO)
{
    SYSCTL -> SRGPIO |= SYSCTL_SRGPIO;
    SYSCTL -> SRGPIO &= ~(SYSCTL_SRGPIO);

}

/********************************************************************************
 * @fn                     - GPIO_InterruptTypeSet
 *
 * @brief                  - This function sets the interrupt type
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s)
 * @param[in]              - Interrupt type
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void GPIO_InterruptTypeSet(uint8_t GPIOx, uint8_t Pinx, uint8_t IntType)
{
    // Get pointer to GPIO port
     GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

    // Configure interrupt type on the pin
    pGPIO -> GPIOIBE = ((IntType & 1) ? (pGPIO -> GPIOIBE | Pinx) : (pGPIO -> GPIOIBE & ~(Pinx)));
    pGPIO -> GPIOIS = ((IntType & 2) ? (pGPIO -> GPIOIS | Pinx) : (pGPIO -> GPIOIS & ~(Pinx)));
    pGPIO -> GPIOIEV = ((IntType & 4) ? (pGPIO -> GPIOIEV | Pinx) : (pGPIO -> GPIOIEV & ~(Pinx)));
}

/********************************************************************************
 * @fn                     - GPIO_EnableInterrupt
 *
 * @brief                  - Enables the specified GPIO interrupts
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s) or interrupt due DMA activity
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void GPIO_EnableInterrupt(uint8_t GPIOx, uint8_t Pinx)
{
     GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

     pGPIO -> GPIOIM |= Pinx;
}

/********************************************************************************
 * @fn                     - GPIO_DisableInterrupt
 *
 * @brief                  - Disables the specified GPIO interrupts
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s) or interrupt due DMA activity
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void GPIO_DisableInterrupt(uint8_t GPIOx, uint8_t Pinx)
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

     pGPIO -> GPIOIM &= ~(Pinx);   
}

/********************************************************************************
 * @fn                     - GPIO_GetInterruptStatus
 *
 * @brief                  - Gets interrupt status for the specified GPIO pin 
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s)
 * 
 * @return                 - 1: interrupt occured, 0: not occured
 * 
 * @Note                   - none
 */
uint8_t GPIO_GetInterruptStatus(uint8_t GPIOx, uint8_t Pinx)
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
    return(pGPIO -> GPIORIS & Pinx);
}

/********************************************************************************
 * @fn                     - GPIO_ClearInterrupt
 *
 * @brief                  - Clears the interrupt for the specified interrupt pin
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s)
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void GPIO_ClearInterrupt(uint8_t GPIOx, uint8_t Pinx)
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
    pGPIO -> GPIOICR |= Pinx;
}

/********************************************************************************
 * @fn                     - GPIO_ReadInputPort
 *
 * @brief                  - This function reads the GPIO port
 * 
 * @param[in]              - GPIO port
 * 
 * @return                 - reads values from 0 to 255 in binary.
 * 
 * @Note                   - none
 */

uint8_t GPIO_ReadInputPort(uint8_t GPIOx) 
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx); 
    return pGPIO -> GPIODATA;
}

/********************************************************************************
 * @fn                     - GPIO_ReadInputPin
 *
 * @brief                  - This function reads from GPIO pin
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin number
 * 
 * @return                 - reads 1 or 0 
 * 
 * @Note                   - none
 */
uint8_t GPIO_ReadInputPin(uint8_t GPIOx, uint8_t Pinx)
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
    return pGPIO->GPIODATA_BITS[Pinx];
}

/********************************************************************************
 * @fn                     - GPIO_WriteOutputPort
 *
 * @brief                  - This function write to GPIO port
 * 
 * @param[in]              - GPIO port
 * @param[in]              - write value from 0 to 255
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void GPIO_WriteOutputPort(uint8_t GPIOx, uint8_t Value){
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

    pGPIO-> GPIODATA = Value;
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

void GPIO_WriteOutputPin(uint8_t GPIOx, uint8_t Pinx, uint8_t Value) {

    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);

    // find the physical pins 
    uint8_t PinNumber = Pinx;
    uint8_t count = 0; 
    while (PinNumber > 1)
    {
        PinNumber >>= 1;
        count++;
    }

    uint8_t data = Value << count;

    pGPIO -> GPIODATA_BITS[Pinx] = data;
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
void GPIO_ToggleOutputPin(uint8_t GPIOx, uint8_t Pinx)
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
    pGPIO->GPIODATA ^= Pinx;
}

/********************************************************************************
 * @fn                     - GPIO_Lock
 *
 * @brief                  - This function locks the GPIOCR register for write access
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - GPIO pin number
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */

void GPIO_Lock(uint8_t GPIOx, uint8_t Pinx)
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
   
    pGPIO -> GPIOLOCK = GPIO_LOCK_M;
    pGPIO -> GPIOCR &= ~(Pinx);
}

/********************************************************************************
 * @fn                     - GPIO_Unlock
 *
 * @brief                  - This function unlocks the GPIOCR register for write access
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - GPIO pin number
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */

void GPIO_Unlock(uint8_t GPIOx, uint8_t Pinx)
{
    GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
    
    pGPIO -> GPIOLOCK = GPIO_LOCK_KEY;
    pGPIO -> GPIOCR |= Pinx;
}

/********************************************************************************
 * @fn                     - GPIO_IRQConfig
 *
 * @brief                  - This function enbales or disables interrupt from processor side
 * @param[in]              - IRQ number
 * @param[in]              - ENABLE or DISABLE macros
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void GPIO_IRQConfig(uint8_t IRQn, uint8_t Ctrl)
{
    if (Ctrl == ENABLE)
    {
        if (IRQn <= 31)
        {
            NVIC_EN0_R |= (1 << IRQn);
        }else if (IRQn > 31 && IRQn < 64)
        {
            NVIC_EN1_R |= (1 << (IRQn % 32));
        }
    }else
    {
        if (IRQn <= 31)
        {
            NVIC_DIS0_R |= (1 << IRQn);
        }else if (IRQn > 31 && IRQn < 64)
        {
            NVIC_DIS1_R |= (1 << (IRQn % 32));
        }
    }
}

/********************************************************************************
 * @fn                     - GPIO_IRQPriorityConfig
 *
 * @brief                  - This function sets interrupt priority from processor side
 * 
 * @param[in]              - IRQ number
 * @param[in]              - IRQ priority
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */

void GPIO_IRQPriorityConfig(uint8_t IRQn, uint32_t IRQPriority)
{
    uint8_t iprx = IRQn / 4;
    uint8_t sections = IRQn % 4;

    uint8_t Shift_amount = (8 * sections) + 5;
    *(NVIC_PRI0_R + iprx) = (IRQPriority << Shift_amount);
}

/********************************************************************************
 * @fn                     - GPIO_IRQHandling
 *
 * @brief                  - This function handles the IRQ
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s)
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
void GPIO_IRQHandling(uint8_t GPIOx, uint8_t Pinx)
{
    // Check if interrupt occurd on the pin
    if (GPIO_GetInterruptStatus(GPIOx, Pinx))
        // Clear interrupt on the pin
        GPIO_ClearInterrupt(GPIOx, Pinx);
}

/********************************************************************************
 * @fn                     - GPIO_SSIType
 *
 * @brief                  - Configure GPIO pin(s) to be SSI type
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s)
 * 
 * @return                 - none
 * 
 * @Note                   - Consult with table 23-5 (GPIO Pins and Alternate Functions) in
 *                         - the datasheet Tiva™ TM4C123GH6PM Microcontroller to see which
 *                         - pins that can be configured as SSI
 */
void GPIO_SSIType(uint8_t GPIOx, uint8_t Pinx)
{
    // Set the pin(s) to AFSEL
    GPIO_ModeSet(GPIOx, Pinx, GPIO_AFSEL);

    // Configure the pad(s)
    GPIO_PadConfig(GPIOx, Pinx, GPIO_DR2R, GPIO_DEN);
}

/********************************************************************************
 * @fn                     - GPIO_UARTType
 *
 * @brief                  - Configure GPIO pin(s) to be UART type
 * 
 * @param[in]              - GPIO port
 * @param[in]              - GPIO pin(s)
 * 
 * @return                 - none
 * 
 * @Note                   - Consult with table 23-5 (GPIO Pins and Alternate Functions) in
 *                         - the datasheet Tiva™ TM4C123GH6PM Microcontroller to see which
 *                         - pins that can be configured as SSI
 */
void GPIO_UARTType(uint8_t GPIOx, uint8_t Pinx)
{
    // Set the pin(s) to AFSEL
    GPIO_ModeSet(GPIOx, Pinx, GPIO_AFSEL);

    // Configure the pad(s)
    GPIO_PadConfig(GPIOx, Pinx, GPIO_DR2R, GPIO_DEN);
}


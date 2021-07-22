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
 * @param[in]              - GPIO port enum
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
void GPIOEnableClk(uint8_t SYSCTL_RCGCGPIO_MODULE)
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
void GPIODisableClk(uint8_t SYSCTL_RCGCGPIO_Portx)
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

void GPIOSelectBus(uint8_t SYSCTL_GPIOHBCTL_PORTx, uint8_t Bus)
{
    if (Bus == AHB)
        SYSCTL -> GPIOHBCTL |= Bus;
     SYSCTL -> GPIOHBCTL &= ~(Bus);
  
}

/********************************************************************************
 * @fn                     - GPIOModeSet
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
 void GPIOModeSet(uint8_t GPIOx, uint8_t Pinx, uint8_t PinIO)
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
 * @fn                     - GPIOPadConfig
 *
 * @brief                  - This function config GPIO pad(s)
 * 
 * @param[in]              -  GPIO port
 * @param[in]              -  GPIO pin(s)
 * @param[in]              -  GPIO type
 * 
 * @return                 - none
 * 
 * @Note                   - none
 */
 void GPIOPadConfig(uint8_t GPIOx, uint8_t Pinx, uint8_t Strength, uint8_t PinType)
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
 * @fn                     - GPIOEnableSLR
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
 void GPIOEnableSLR(uint8_t GPIOx, uint8_t Pinx)
 {
      GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
      
      pGPIO -> GPIOSLR |= Pinx;
 }

 /********************************************************************************
 * @fn                     - GPIODisableSLR
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
 void GPIODisableSLR(uint8_t GPIOx, uint8_t Pinx)
 {
     GPIO_RegDef_t *pGPIO = GPIO_Get_Port(GPIOx);
      
    pGPIO -> GPIOSLR &= ~(Pinx);
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
    
    uint8_t temp;

    //Check if port and pin number is valid
  /*  if(!GPIO_Check_Pin(pGPIOHandle->GPIOx, pGPIOHandle->GPIO_PinConfig.GPIO_PinDir))
      return FALSE; */

    //1. Configure GPIO direction
   /* temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinDir << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODIR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIODIR |= temp;

    //2. Confgure Alternate function
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_AFSEL << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOAFSEL &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOAFSEL |= temp;

    //3. Confgure 2mA drive
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DR2R << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODR2R &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIODR2R |= temp;

    //4. Confgure 4mA drive
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DR4R << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODR4R &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIODR4R |= temp;

    //5. Confgure 8mA drive
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DR8R << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODR8R &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 
    pGPIOHandle -> GPIOx -> GPIODR8R |= temp;

    //6. Confgure open drain
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_OODR << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOODR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOODR |= temp;

    //7. Confgure pull up
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PUR << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOPUR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOPUR |= temp;

    //8. Confgure pull down
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PDR << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOPDR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOPDR |= temp;

    //10. Confgure digital function
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DEN << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODEN &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIODEN |= temp;

    //12. Confgure analog funtion
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_AMSEL << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOAMSEL &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 
    pGPIOHandle -> GPIOx -> GPIOAMSEL |= temp; */

    //13. Confgure port control
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PCTL << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOPCTL &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOPCTL |= temp;

  /*  //14. Confgure adc trigger
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_ADCCTL << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOADCCTL &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIOADCCTL |= temp;

    //15. Confgure dma trigger
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_DMACTL << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIODMACTL &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle -> GPIOx -> GPIODMACTL |= temp;*/

    return TRUE;
}

/********************************************************************************
 * @fn                     - GPIO_DeInit
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
 * @fn                     - GPIO_InterruptInit
 *
 * @brief                  - This function initialize GPIO interrupt
 * 
 * @param[in]              - base address of the gpio peripheral
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */

uint8_t GPIO_InterruptInit(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp;

    // Prevent false interrupts on edge sensitive detection
    if(pGPIOHandle->GPIO_PinConfig.GPIO_IS == GPIO_IS_EDGE)
    {   
        // Mask GPIOM register
        pGPIOHandle -> GPIOx -> GPIOIM &= GPIO_IM_GPIO_M;
        
        // Configure Interrupt sense
        temp = 0;
        temp = pGPIOHandle -> GPIO_PinConfig.GPIO_IS << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
        pGPIOHandle -> GPIOx -> GPIOIS |= temp;
        
        // Configure Interrupt both edge
        temp = 0;
        temp = pGPIOHandle -> GPIO_PinConfig.GPIO_IBE << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
        pGPIOHandle -> GPIOx -> GPIOIBE |= temp;
        
        // Clear GPIORIS
        pGPIOHandle -> GPIOx-> GPIOICR = GPIO_ICR_GPIO_M;

        // Unmask the port
        pGPIOHandle -> GPIOx -> GPIOIM |= GPIO_IM_GPIO_S;

        // Configure Interrupt Event
        temp = 0;
        temp = pGPIOHandle -> GPIO_PinConfig.GPIO_IEV << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;
        pGPIOHandle -> GPIOx -> GPIOIEV |= temp;
    }

    pGPIOHandle -> GPIOx -> GPIOIM &= GPIO_IM_GPIO_S;

    temp = 0;
    temp = pGPIOHandle -> GPIO_PinConfig.GPIO_IS << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOIS |= temp;

    temp = 0;
    temp = pGPIOHandle -> GPIO_PinConfig.GPIO_IBE << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOIBE |= temp;

    temp = 0;
    temp = pGPIOHandle -> GPIO_PinConfig.GPIO_IEV << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle -> GPIOx -> GPIOIEV |= temp;

    //Clear interrupt
    pGPIOHandle -> GPIOx -> GPIOICR |= GPIO_ICR_GPIO_M;

    // Unmask the port
    pGPIOHandle -> GPIOx -> GPIOIM |= (1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
    

    return TRUE;
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
 * @param[in]              - IRQ priority
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
 * @brief                  - This function enbales or disables peripheral clock for the given GPIO port
 * 
 * @param[in]              - base address of the gpio peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * 
 * @return                 - TRUE or FALSE macros
 * 
 * @Note                   - none
 */
void GPIO_IRQHandling(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    // Check if interrupt occurd on the pin
    if (pGPIOx -> GPIORIS & (1 << PinNumber))
        // Clear interrupt on the pin
        pGPIOx -> GPIOICR |= (1 << PinNumber);
}


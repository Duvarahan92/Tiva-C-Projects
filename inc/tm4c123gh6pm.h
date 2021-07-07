//*****************************************************************************
//
// tm4c123gh6pm.h - TM4C123GH6PM Register Definitions
//
//
//*****************************************************************************

#ifndef __TM4C123GH6PM_H__
#define __TM4C123GH6PM_H__

#include <stdint.h>

//*****************************************************************************
//
// Interrupt assignments
//
//*****************************************************************************
#define INT_GPIOA               16          // GPIO Port A
#define INT_GPIOB               17          // GPIO Port B
#define INT_GPIOC               18          // GPIO Port C
#define INT_GPIOD               19          // GPIO Port D
#define INT_GPIOE               20          // GPIO Port E
#define INT_UART0               21          // UART0
#define INT_UART1               22          // UART1
#define INT_SSI0                23          // SSI0
#define INT_I2C0                24          // I2C0
#define INT_PWM0_FAULT          25          // PWM0 Fault
#define INT_PWM0_0              26          // PWM0 Generator 0
#define INT_PWM0_1              27          // PWM0 Generator 1
#define INT_PWM0_2              28          // PWM0 Generator 2
#define INT_QEI0                29          // QEI0
#define INT_ADC0SS0             30          // ADC0 Sequence 0
#define INT_ADC0SS1             31          // ADC0 Sequence 1
#define INT_ADC0SS2             32          // ADC0 Sequence 2
#define INT_ADC0SS3             33          // ADC0 Sequence 3
#define INT_WATCHDOG            34          // Watchdog Timers 0 and 1
#define INT_TIMER0A             35          // 16/32-Bit Timer 0A
#define INT_TIMER0B             36          // 16/32-Bit Timer 0B
#define INT_TIMER1A             37          // 16/32-Bit Timer 1A
#define INT_TIMER1B             38          // 16/32-Bit Timer 1B
#define INT_TIMER2A             39          // 16/32-Bit Timer 2A
#define INT_TIMER2B             40          // 16/32-Bit Timer 2B
#define INT_COMP0               41          // Analog Comparator 0
#define INT_COMP1               42          // Analog Comparator 1
#define INT_SYSCTL              44          // System Control
#define INT_FLASH               45          // Flash Memory Control and EEPROM
                                            // Control
#define INT_GPIOF               46          // GPIO Port F
#define INT_UART2               49          // UART2
#define INT_SSI1                50          // SSI1
#define INT_TIMER3A             51          // 16/32-Bit Timer 3A
#define INT_TIMER3B             52          // Timer 3B
#define INT_I2C1                53          // I2C1
#define INT_QEI1                54          // QEI1
#define INT_CAN0                55          // CAN0
#define INT_CAN1                56          // CAN1
#define INT_HIBERNATE           59          // Hibernation Module
#define INT_USB0                60          // USB
#define INT_PWM0_3              61          // PWM Generator 3
#define INT_UDMA                62          // uDMA Software
#define INT_UDMAERR             63          // uDMA Error
#define INT_ADC1SS0             64          // ADC1 Sequence 0
#define INT_ADC1SS1             65          // ADC1 Sequence 1
#define INT_ADC1SS2             66          // ADC1 Sequence 2
#define INT_ADC1SS3             67          // ADC1 Sequence 3
#define INT_SSI2                73          // SSI2
#define INT_SSI3                74          // SSI3
#define INT_UART3               75          // UART3
#define INT_UART4               76          // UART4
#define INT_UART5               77          // UART5
#define INT_UART6               78          // UART6
#define INT_UART7               79          // UART7
#define INT_I2C2                84          // I2C2
#define INT_I2C3                85          // I2C3
#define INT_TIMER4A             86          // 16/32-Bit Timer 4A
#define INT_TIMER4B             87          // 16/32-Bit Timer 4B
#define INT_TIMER5A             108         // 16/32-Bit Timer 5A
#define INT_TIMER5B             109         // 16/32-Bit Timer 5B
#define INT_WTIMER0A            110         // 32/64-Bit Timer 0A
#define INT_WTIMER0B            111         // 32/64-Bit Timer 0B
#define INT_WTIMER1A            112         // 32/64-Bit Timer 1A
#define INT_WTIMER1B            113         // 32/64-Bit Timer 1B
#define INT_WTIMER2A            114         // 32/64-Bit Timer 2A
#define INT_WTIMER2B            115         // 32/64-Bit Timer 2B
#define INT_WTIMER3A            116         // 32/64-Bit Timer 3A
#define INT_WTIMER3B            117         // 32/64-Bit Timer 3B
#define INT_WTIMER4A            118         // 32/64-Bit Timer 4A
#define INT_WTIMER4B            119         // 32/64-Bit Timer 4B
#define INT_WTIMER5A            120         // 32/64-Bit Timer 5A
#define INT_WTIMER5B            121         // 32/64-Bit Timer 5B
#define INT_SYSEXC              122         // System Exception (imprecise)
#define INT_PWM1_0              150         // PWM1 Generator 0
#define INT_PWM1_1              151         // PWM1 Generator 1
#define INT_PWM1_2              152         // PWM1 Generator 2
#define INT_PWM1_3              153         // PWM1 Generator 3
#define INT_PWM1_FAULT          154         // PWM1 Fault

//*****************************************************************************
//
// APB and AHB Bus Peripheral base addresses
//
//*****************************************************************************
#define APB_BASEADDR            0x40000000U
#define AHB_BASEADDR            0x40050000U

//*****************************************************************************
//
// Watchdog Timer Base Addresses
//
//*****************************************************************************
#define WATCHDOG0_BASEADDR      (APB_BASEADDR + 0x0000)
#define WATCHDOG1_BASEADDR      (APB_BASEADDR + 0x1000)

//*****************************************************************************
//
// Watchdog Timer Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t WDTLOAD;                 /*!< Watchdog Load Register,                                                      Address offset: 0x000 */
  volatile uint32_t WDTVALUE;                /*!< Watchdog Value Register,                                                     Address offset: 0x004 */
  volatile uint32_t WDTCTL;                  /*!< Watchdog Control Register                                                    Address offset: 0x008 */    
  volatile uint32_t WDTICR;                  /*!< Watchdog Interrupt Clear Register                                            Address offset: 0x00C */
  volatile uint32_t WDTRIS;                  /*!< Watchdog Raw Interrupt Status Register                                       Address offset: 0x010 */
  volatile uint32_t WDTMIS;                  /*!< Watchdog Masked Interrupt Status Register                                    Address offset: 0x014 */
  volatile uint32_t RESERVED0[256];
  volatile uint32_t WDTTEST;                 /*!< Watchdog Watchdog Test Register                                              Address offset: 0x418 */
  volatile uint32_t RESERVED1[505];
  volatile uint32_t WDTLOCK;                 /*!< Watchdog Watchdog Lock Register                                              Address offset: 0xC00 */
  
}WATCHDOG_RegDef_t;

//*****************************************************************************
//
// Watchdog Base Addresses Type Casted to WATCHDOG_RegDef*
//
//*****************************************************************************
#define WATCHDOG0               ((WATCHDOG_RegDef_t*) WATCHDOG0_BASEADDR)
#define WATCHDOG1               ((WATCHDOG_RegDef_t*) WATCHDOG1_BASEADDR)

//*****************************************************************************
//
// GPIO Base Addresses
//
//*****************************************************************************
#define GPIOA_BASEADDR          (APB_BASEADDR + 0x4000)
#define GPIOB_BASEADDR          (APB_BASEADDR + 0x5000)
#define GPIOC_BASEADDR          (APB_BASEADDR + 0x6000)
#define GPIOD_BASEADDR          (APB_BASEADDR + 0x7000)
#define GPIOE_BASEADDR          (APB_BASEADDR + 0x24000)
#define GPIOF_BASEADDR          (APB_BASEADDR + 0x25000)

#define GPIOA_AHB_BASEADDR          (AHB_BASEADDR + 0x58000)
#define GPIOB_AHB_BASEADDR          (AHB_BASEADDR + 0x59000)
#define GPIOC_AHB_BASEADDR          (AHB_BASEADDR + 0x5A000)
#define GPIOD_AHB_BASEADDR          (AHB_BASEADDR + 0x5B000)
#define GPIOE_AHB_BASEADDR          (AHB_BASEADDR + 0x5C000)
#define GPIOF_AHB_BASEADDR          (AHB_BASEADDR + 0x5D000)

//*****************************************************************************
//
// GPIO Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t GPIODATA_BITS [255];     /*!< Read/Write Data Using Address Mask,                                          Address offset: 0x000 */
  volatile uint32_t GPIODATA;                /*!< GPIO Data,                                                                   Address offset: 0x3FC */
  volatile uint32_t GPIODIR;                 /*!< GPIO Direction,                                                              Address offset: 0x400 */
  volatile uint32_t GPIOIS;                  /*!< GPIO Interrupt Sense,                                                        Address offset: 0x404 */
  volatile uint32_t GPIOIBE;                 /*!< GPIO Interrupt Both Edges,                                                   Address offset: 0x408 */
  volatile uint32_t GPIOIEV;                 /*!< GPIO Interrupt Event,                                                        Address offset: 0x40C */
  volatile uint32_t GPIOIM;                  /*!< GPIO Interrupt Mask,                                                         Address offset: 0x410 */
  volatile uint32_t GPIORIS;                 /*!< GPIO Raw Interrupt Status,                                                   Address offset: 0x414 */
  volatile uint32_t GPIOMIS;                 /*!< GPIO Masked Interrupt Status,                                                Address offset: 0x418 */
  volatile uint32_t GPIOICR;                 /*!< GPIO Interrupt Clear,                                                        Address offset: 0x41C */
  volatile uint32_t GPIOAFSEL;               /*!< GPIO Alternate Function Select,                                              Address offset: 0x420 */
  volatile uint32_t RESERVERD0[55];
  volatile uint32_t GPIODR2R;                /*!< GPIO 2-mA Drive Select,                                                      Address offset: 0x500 */ 
  volatile uint32_t GPIODR4R;                /*!< GPIO 4-mA Drive Select,                                                      Address offset: 0x504 */ 
  volatile uint32_t GPIODR8R;                /*!< GPIO 8-mA Drive Select,                                                      Address offset: 0x508 */ 
  volatile uint32_t GPIOODR;                 /*!< GPIO Open Drain Select,                                                      Address offset: 0x50C */ 
  volatile uint32_t GPIOPUR;                 /*!< GPIO Pull-Up Select,                                                         Address offset: 0x510 */ 
  volatile uint32_t GPIOPDR;                 /*!< GPIO Pull-Down Select,                                                       Address offset: 0x514 */ 
  volatile uint32_t GPIOSLR;                 /*!< GPIO Slew Rate Control Select,                                               Address offset: 0x518 */ 
  volatile uint32_t GPIODEN;                /*!< GPIO Digital Enable,                                                         Address offset: 0x51C */ 
  volatile uint32_t GPIOLOCK;                /*!< GPIO Lock,                                                                   Address offset: 0x520 */ 
  volatile uint32_t GPIOCR;                  /*!< GPIO Commit,                                                                 Address offset: 0x524 */ 
  volatile uint32_t GPIOAMSEL;               /*!< GPIO Analog Mode Select,                                                     Address offset: 0x528 */ 
  volatile uint32_t GPIOPCTL;                /*!< GPIO Port Control,                                                           Address offset: 0x52C */ 
  volatile uint32_t GPIOADCCTL;              /*!< GPIO ADC Control,                                                            Address offset: 0x530 */ 
  volatile uint32_t GPIODMACTL;              /*!< GPIO DMA Control,                                                            Address offset: 0x534 */ 

}GPIO_RegDef_t;

//*****************************************************************************
//
// GPIO Base Addresses Type Casted to GPIO_RegDef_t*
//
//*****************************************************************************

#define GPIOA                    ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                    ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                    ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                    ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                    ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                    ((GPIO_RegDef_t*) GPIOF_BASEADDR)

#define GPIOA_AHB               ((GPIO_RegDef_t*) GPIOA_AHB_BASEADDR)
#define GPIOB_AHB               ((GPIO_RegDef_t*) GPIOB_AHB_BASEADDR)
#define GPIOC_AHB               ((GPIO_RegDef_t*) GPIOC_AHB_BASEADDR)
#define GPIOD_AHB               ((GPIO_RegDef_t*) GPIOD_AHB_BASEADDR)
#define GPIOE_AHB               ((GPIO_RegDef_t*) GPIOE_AHB_BASEADDR)
#define GPIOF_AHB               ((GPIO_RegDef_t*) GPIOF_AHB_BASEADDR)

//*****************************************************************************
//
// SSI Base Addresses
//
//*****************************************************************************

#define SSI0_BASEADDR           (APB_BASEADDR + 0x8000)
#define SSI1_BASEADDR           (APB_BASEADDR + 0x9000)
#define SSI2_BASEADDR           (APB_BASEADDR + 0xA000)
#define SSI3_BASEADDR           (APB_BASEADDR + 0xB000)


//*****************************************************************************
//
// SSI Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t SSICR0;                  /*!< SSI Control 0,                                                               Address offset: 0x000 */ 
  volatile uint32_t SSICR1;                  /*!< SSI Control 1,                                                               Address offset: 0x004 */
  volatile uint32_t SSIDR;                   /*!< SSI DATA,                                                                    Address offset: 0x008 */
  volatile uint32_t SSISR;                   /*!< SSI Status,                                                                  Address offset: 0x00C */
  volatile uint32_t SSICPSR;                 /*!< SSI Clock Prescale,                                                          Address offset: 0x010 */
  volatile uint32_t SSIIM;                   /*!< SSI Interrupt Mask,                                                          Address offset: 0x014 */
  volatile uint32_t SSIRIS;                  /*!< SSI Raw Interrupt Mask,                                                      Address offset: 0x018 */
  volatile uint32_t SSIMIS;                  /*!< SSI Masked Interrupt Status,                                                 Address offset: 0x01C */
  volatile uint32_t SSIICR;                  /*!< SSI Interrupt Clear,                                                         Address offset: 0x020 */
  volatile uint32_t SSIDMACTL;               /*!< SSI DMA Control,                                                             Address offset: 0x024 */
  volatile uint32_t Reserved[1000];
  volatile uint32_t SSICC;                   /*!< SSI Clock Configuration,                                                     Address offset: 0xFC8 */

}SSI_RegDef_t;

//*****************************************************************************
//
// SSI Base Addresses Type Casted to SSI_RegDef_t*
//
//*****************************************************************************

#define SSI0                     ((SSI_RegDef*) SSI0_BASEADDR)
#define SSI1                     ((SSI_RegDef*) SSI1_BASEADDR)
#define SSI2                     ((SSI_RegDef*) SSI2_BASEADDR)
#define SSI3                     ((SSI_RegDef*) SSI3_BASEADDR)

//*****************************************************************************
//
// UART Base Addresses
//
//*****************************************************************************

#define UART0_BASEADDR                    (APB_BASEADDR + 0xC000)
#define UART1_BASEADDR                    (APB_BASEADDR + 0xD000)
#define UART2_BASEADDR                    (APB_BASEADDR + 0xE000)
#define UART3_BASEADDR                    (APB_BASEADDR + 0xF000)
#define UART4_BASEADDR                    (APB_BASEADDR + 0x10000)
#define UART5_BASEADDR                    (APB_BASEADDR + 0x11000)
#define UART6_BASEADDR                    (APB_BASEADDR + 0x12000)
#define UART7_BASEADDR                    (APB_BASEADDR + 0x13000)

//*****************************************************************************
//
// UART Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t UARTDR;                  /*!< UART Data                                                                    Addresses offset: 0x000 */ 
  volatile uint32_t UART_RSR_ECR;            /*!< UART Recevice Status/Error Clear                                             Addresses offset: 0x004 */
  volatile uint32_t RESERVED0[4];
  volatile uint32_t UARTFR;                  /*!< UART Flag                                                                    Addresses offset: 0x018 */
  volatile uint32_t RESERVED1;
  volatile uint32_t UARTILPR;                /*!< UART lrDa Low-Power Register                                                 Addresses offset: 0x020 */
  volatile uint32_t UARTBRD;                 /*!< UART Interger Baud-Rate Divisor                                              Addresses offset: 0x024 */
  volatile uint32_t UARTFBRD;                /*!< UART Fractional Baud-Rate Divisor                                            Addresses offset: 0x028 */
  volatile uint32_t UARTLCRH;                /*!< UART Line Control                                                            Addresses offset: 0x02C */
  volatile uint32_t UARTCTL;                 /*!< UART Control                                                                 Addresses offset: 0x030 */
  volatile uint32_t UARTIFLS;                /*!< UART Interrupt FIFO Level Select                                             Addresses offset: 0x034 */
  volatile uint32_t UARTIM;                  /*!< UART Interrupt Mask                                                          Addresses offset: 0x038 */
  volatile uint32_t UARTRIS;                 /*!< UART Raw Interrupt Status                                                    Addresses offset: 0x03C */
  volatile uint32_t UARTMIS;                 /*!< UART Masked Interrupt Status                                                 Addresses offset: 0x040 */
  volatile uint32_t UARTICR;                 /*!< UART Interrupt Clear                                                         Addresses offset: 0x044 */
  volatile uint32_t UARTDMACTL;              /*!< UART DMA Control                                                             Addresses offset: 0x048 */
  volatile uint32_t RESERVED2[22];
  volatile uint32_t UART9BITADDR;            /*!< UART 9-Bit Self Address                                                      Addresses offset: 0x0A4 */
  volatile uint32_t UART9BITAMASK;           /*!< UART 9-Bit Self Address Mask                                                 Addresses offset: 0x0A8 */
  volatile uint32_t RESERVED3[965];
  volatile uint32_t UARTPP;                  /*!< UART Peripheral Properties                                                   Addresses offset: 0xFC0 */
  volatile uint32_t RESERVED4;
  volatile uint32_t UARTCC;                  /*!< UART Clock Configuration                                                     Addresses offset: 0xFC8 */
 
}UART_RegDef_t;

//*****************************************************************************
//
// UART Base Addresses Type Casted to UART_RegDef_t*
//
//*****************************************************************************

#define UART0                   ((UART_RegDef_t*) UART0_BASEADDR)
#define UART1                   ((UART_RegDef_t*) UART1_BASEADDR)
#define UART2                   ((UART_RegDef_t*) UART2_BASEADDR)
#define UART3                   ((UART_RegDef_t*) UART3_BASEADDR)
#define UART4                   ((UART_RegDef_t*) UART4_BASEADDR)
#define UART5                   ((UART_RegDef_t*) UART5_BASEADDR)
#define UART6                   ((UART_RegDef_t*) UART6_BASEADDR)
#define UART7                   ((UART_RegDef_t*) UART7_BASEADDR)

//*****************************************************************************
//
// I2C Base Addresses
//
//*****************************************************************************

#define I2C0_BASEADDR           (APB_BASEADDR + 0x20000)
#define I2C1_BASEADDR           (APB_BASEADDR + 0x21000)
#define I2C2_BASEADDR           (APB_BASEADDR + 0x22000)
#define I2C3_BASEADDR           (APB_BASEADDR + 0x23000)

//*****************************************************************************
//
// I2C Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t I2CMSA;                  /*!< I2C Master Slave Address                                                     Addresses offset: 0x000 */
  volatile uint32_t I2CMCS;                  /*!< I2C Master Control/Status                                                    Addresses offset: 0x004 */
  volatile uint32_t I2CMDR;                  /*!< I2C Master Data                                                              Addresses offset: 0x008 */
  volatile uint32_t I2CMTPR;                 /*!< I2C Master Timer Period                                                      Addresses offset: 0x00C */
  volatile uint32_t I2CMIMR;                 /*!< I2C Master Interrupt Mask                                                    Addresses offset: 0x010 */
  volatile uint32_t I2CMRIS;                 /*!< I2C Master Raw Interrupt Status                                              Addresses offset: 0x014 */
  volatile uint32_t I2CMMIS;                 /*!< I2C Master Masked Interrupt Status                                           Addresses offset: 0x018 */
  volatile uint32_t I2CMICR;                 /*!< I2C Master Interrupt Clear                                                   Addresses offset: 0x01C */
  volatile uint32_t I2CMCR;                  /*!< I2C Master Configuration                                                     Addresses offset: 0x020 */
  volatile uint32_t I2CMCLKOCNT;             /*!< I2C Master Clock Low Timeout Count                                           Addresses offset: 0x024 */
  volatile uint32_t RESERVED0;
  volatile uint32_t I2CMBMON;                /*!< I2C Master Bus Monitor                                                       Addresses offset: 0x02C */
  volatile uint32_t RESERVED1[2];
  volatile uint32_t I2CMCR2;                 /*!< I2C Master Configuration 2                                                   Addresses offset: 0x038 */
  volatile uint32_t RESERVED2[497];
  volatile uint32_t I2CSOAR;                 /*!< I2C Slave Own Address                                                        Addresses offset: 0x800 */
  volatile uint32_t I2CSCSR;                 /*!< I2C Slave Control/Status                                                     Addresses offset: 0x804 */
  volatile uint32_t I2CSDR;                  /*!< I2C Slave Data                                                               Addresses offset: 0x808 */
  volatile uint32_t I2CSIMR;                 /*!< I2C Slave Interrupt Mask                                                     Addresses offset: 0x80C */
  volatile uint32_t I2CSRIS;                 /*!< I2C Slave Raw Interrupt Status                                               Addresses offset: 0x810 */
  volatile uint32_t I2CSMIS;                 /*!< I2C Slave Masked Interrupt Status                                            Addresses offset: 0x814 */
  volatile uint32_t I2CSICR;                 /*!< I2C Slave Interrupt Clear                                                    Addresses offset: 0x818 */
  volatile uint32_t I2CSOAR2;                /*!< I2C Slave Own Address 2                                                      Addresses offset: 0x81C */
  volatile uint32_t I2CSACKCTL;              /*!< I2C Slave ACK Control                                                        Addresses offset: 0x820 */
  volatile uint32_t RESERVED3[487];
  volatile uint32_t I2CPP;                   /*!< I2C Peripheral Properties                                                    Addresses offset: 0xFC0 */
  volatile uint32_t I2CPC;                   /*!< I2C Peripheral Configuration                                                 Addresses offset: 0xFC4 */

}I2C_RegDef_t;

//*****************************************************************************
//
// I2C Base Addresses Type Casted to I2C_RegDef_t* 
//
//*****************************************************************************

#define I2C0                    ((I2C_RegDef_t*) I2C0_BASEADDR)
#define I2C1                    ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2                    ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3                    ((I2C_RegDef_t*) I2C3_BASEADDR)

//*****************************************************************************
//
// PWM Base Addresses
//
//*****************************************************************************

#define PWM0_BASEADDR                    (APB_BASEADDR + 0x28000)
#define PWM1_BASEADDR                    (APB_BASEADDR + 0x29000)

//*****************************************************************************
//
// PWM Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t PWMCTL;                  /*!< PWM Master Control                                                           Addresses offset: 0x000 */
  volatile uint32_t PWMSYNC;                 /*!< PWM Time Base Sync                                                           Addresses offset: 0x004 */
  volatile uint32_t PWMENABLE;               /*!< PWM Output Enable                                                            Addresses offset: 0x008 */
  volatile uint32_t PWMINVERT;               /*!< PWM Output Inversion                                                         Addresses offset: 0x00C */
  volatile uint32_t PWMFAULT;                /*!< PWM Output Fault                                                             Addresses offset: 0x010 */
  volatile uint32_t PWMINTEN;                /*!< PWM Interrupt Enable                                                         Addresses offset: 0x014 */
  volatile uint32_t PWMRIS;                  /*!< PWM Raw Interrupt Status                                                     Addresses offset: 0x018 */
  volatile uint32_t PWMISC;                  /*!< PWM Interrupt Status and Clear                                               Addresses offset: 0x01C */
  volatile uint32_t PWMSTATUS;               /*!< PWM Status                                                                   Addresses offset: 0x020 */
  volatile uint32_t PWMFAULTVAL;             /*!< PWM Fault Condition Value                                                    Addresses offset: 0x024 */
  volatile uint32_t PWMENUPD;                /*!< PWM Enable Update                                                            Addresses offset: 0x028 */
  volatile uint32_t RESERVED0[5];
  volatile uint32_t PWM0CTL;                 /*!< PWM0 Control                                                                 Addresses offset: 0x040 */
  volatile uint32_t PWM0INTEN;               /*!< PWM0 Interrupt and Trigger Enable                                            Addresses offset: 0x044 */
  volatile uint32_t PWM0RIS;                 /*!< PWM0 Raw Interrupt Status                                                    Addresses offset: 0x048 */
  volatile uint32_t PWM0ISC;                 /*!< PWM0 Interrupt Status and Clear                                              Addresses offset: 0x04C */
  volatile uint32_t PWM0LOAD;                /*!< PWM0 Load                                                                    Addresses offset: 0x050 */
  volatile uint32_t PWM0COUNT;               /*!< PWM0 Counter                                                                 Addresses offset: 0x054 */
  volatile uint32_t PWM0CMPA;                /*!< PWM0CMPA                                                                     Addresses offset: 0x058 */
  volatile uint32_t PWM0CMPB;                /*!< PWM0 Compare B                                                               Addresses offset: 0x05C */
  volatile uint32_t PWM0GENA;                /*!< PWM0 Generator A Control                                                     Addresses offset: 0x060 */
  volatile uint32_t PWM0GENB;                /*!< PWM0 Generator B Control                                                     Addresses offset: 0x064 */
  volatile uint32_t PWM0DBCTL;               /*!< PWM0 Dead-Band Control                                                       Addresses offset: 0x068 */
  volatile uint32_t PWM0DBRISE;              /*!< PWM0 Dead-Band Rising-Edge Delay                                             Addresses offset: 0x06C */
  volatile uint32_t PWM0DBFALL;              /*!< PWM0 Dead-Band Falling-Edge-Delay                                            Addresses offset: 0x070 */
  volatile uint32_t PWM0FLTSRC0;             /*!< PWM0 Fault Source 0                                                          Addresses offset: 0x074 */
  volatile uint32_t PWM0FLTSRC1;             /*!< PWM0 Fault Source 1                                                          Addresses offset: 0x078 */
  volatile uint32_t PWM0MINFLTPER;           /*!< PWM0 Minimum Fault Period                                                    Addresses offset: 0x07C */
  volatile uint32_t PWM1CTL;                 /*!< PWM1 Control                                                                 Addresses offset: 0x080 */
  volatile uint32_t PWM1INTEN;               /*!< PWM1 Interrupt and Trigger Enable                                            Addresses offset: 0x084 */
  volatile uint32_t PWM1RIS;                 /*!< PWM1 Raw Interrupt Status                                                    Addresses offset: 0x088 */
  volatile uint32_t PWM1ISC;                 /*!< PWM1 Interrupt Status and Clear                                              Addresses offset: 0x08C */
  volatile uint32_t PWM1LOAD;                /*!< PWM1 Load                                                                    Addresses offset: 0x090 */
  volatile uint32_t PWM1COUNT;               /*!< PWM1 Counter                                                                 Addresses offset: 0x094 */
  volatile uint32_t PWM1CMPA;                /*!< PWM1 Compare A                                                               Addresses offset: 0x098 */
  volatile uint32_t PWM1CMPB;                /*!< PWM1 Compare B                                                               Addresses offset: 0x09C */
  volatile uint32_t PWM1GENA;                /*!< PWM1 Generator A Control                                                     Addresses offset: 0x0A0 */
  volatile uint32_t PWM1GENB;                /*!< PWM1 Generator B Control                                                     Addresses offset: 0x0A4 */
  volatile uint32_t PWM1DBCTL;               /*!< PWM1 Dead-Band Control                                                       Addresses offset: 0x0A8 */
  volatile uint32_t PWM1DBRISE;              /*!< PWM1 Dead-Band Rising-Edge Delay                                             Addresses offset: 0x0AC */
  volatile uint32_t PWM1DBFALL;              /*!< PWM1 Dead-Band Falling-Edge-Delay                                            Addresses offset: 0x0B0 */
  volatile uint32_t PWM1FLTSRC0;             /*!< PWM1 Fault Source 0                                                          Addresses offset: 0x0B4 */
  volatile uint32_t PWM1FLTSRC1;             /*!< PWM1 Fault Source 1                                                          Addresses offset: 0x0B8 */
  volatile uint32_t PWM1MINFLTPER;           /*!< PWM1 Minimum Fault Period                                                    Addresses offset: 0x0BC */
  volatile uint32_t PWM2CTL;                 /*!< PWM2 Control                                                                 Addresses offset: 0x0C0 */
  volatile uint32_t PWM2INTEN;               /*!< PWM2 Interrupt and Trigger Enable                                            Addresses offset: 0x0C4 */
  volatile uint32_t PWM2RIS;                 /*!< PWM2 Raw Interrupt Status                                                    Addresses offset: 0x0C8 */
  volatile uint32_t PWM2ISC;                 /*!< PWM2 Interrupt Status and Clear                                              Addresses offset: 0x0CC */
  volatile uint32_t PWM2LOAD;                /*!< PWM2 Load                                                                    Addresses offset: 0x0D0 */
  volatile uint32_t PWM2COUNT;               /*!< PWM2 Counter                                                                 Addresses offset: 0x0D4 */
  volatile uint32_t PWM2CMPA;                /*!< PWM2 Compare A                                                               Addresses offset: 0x0D8 */
  volatile uint32_t PWM2CMPB;                /*!< PWM2 Compare B                                                               Addresses offset: 0x0DC */
  volatile uint32_t PWM2GENA;                /*!< PWM2 Generator A Control                                                     Addresses offset: 0x0E0 */
  volatile uint32_t PWM2GENB;                /*!< PWM2 Generator B Control                                                     Addresses offset: 0x0E4 */
  volatile uint32_t PWM2DBCTL;               /*!< PWM2 Dead-Band Control                                                       Addresses offset: 0x0E8 */
  volatile uint32_t PWM2DBRISE;              /*!< PWM2 Dead-Band Rising-Edge Delay                                             Addresses offset: 0x0EC */
  volatile uint32_t PWM2DBFALL;              /*!< PWM2 Dead-Band Falling-Edge-Delay                                            Addresses offset: 0x0F0 */
  volatile uint32_t PWM2FLTSRC0;             /*!< PWM2 Fault Source 0                                                          Addresses offset: 0x0F4 */
  volatile uint32_t PWM2FLTSRC1;             /*!< PWM2 Fault Source 1                                                          Addresses offset: 0x0F8 */
  volatile uint32_t PWM2MINFLTPER;           /*!< PWM2 Minimum Fault Period                                                    Addresses offset: 0x0FC */
  volatile uint32_t PWM3CTL;                 /*!< PWM3 Control                                                                 Addresses offset: 0x100 */
  volatile uint32_t PWM3INTEN;               /*!< PWM3 Interrupt and Trigger Enable                                            Addresses offset: 0x104 */
  volatile uint32_t PWM3RIS;                 /*!< PWM3 Raw Interrupt Status                                                    Addresses offset: 0x108 */
  volatile uint32_t PWM3ISC;                 /*!< PWM3 Interrupt Status and Clear                                              Addresses offset: 0x10C */
  volatile uint32_t PWM3LOAD;                /*!< PWM3 Load                                                                    Addresses offset: 0x110 */
  volatile uint32_t PWM3COUNT;               /*!< PWM3 Counter                                                                 Addresses offset: 0x114 */
  volatile uint32_t PWM3CMPA;                /*!< PWM3 Compare A                                                               Addresses offset: 0x118 */
  volatile uint32_t PWM3CMPB;                /*!< PWM3 Compare B                                                               Addresses offset: 0x11C */
  volatile uint32_t PWM3GENA;                /*!< PWM3 Generator A Control                                                     Addresses offset: 0x120 */
  volatile uint32_t PWM3GENB;                /*!< PWM3 Generator B Control                                                     Addresses offset: 0x124 */
  volatile uint32_t PWM3DBCTL;               /*!< PWM3 Dead-Band Control                                                       Addresses offset: 0x128 */
  volatile uint32_t PWM3DBRISE;              /*!< PWM3 Dead-Band Rising-Edge Delay                                             Addresses offset: 0x12C */
  volatile uint32_t PWM3DBFALL;              /*!< PWM3 Dead-Band Falling-Edge-Delay                                            Addresses offset: 0x130 */
  volatile uint32_t PWM3FLTSRC0;             /*!< PWM3 Fault Source 0                                                          Addresses offset: 0x134 */
  volatile uint32_t PWM3FLTSRC1;             /*!< PWM3 Fault Source 1                                                          Addresses offset: 0x138 */
  volatile uint32_t PWM3MINFLTPER;           /*!< PWM3 Minimum Fault Period                                                    Addresses offset: 0x13C */
  volatile uint32_t RESERVED1[432];
  volatile uint32_t PWM0FLTSEN;              /*!< PWM0 Fault Pin Logic Sense                                                   Addresses offset: 0x800 */
  volatile uint32_t PWM0FLTSTAT0;            /*!< PWM0 Fault Status 0                                                          Addresses offset: 0x804 */
  volatile uint32_t PWM0FLTSTAT1;            /*!< PWM0 Fault Status 1                                                          Addresses offset: 0x808 */
  volatile uint32_t RESERVED2[29];
  volatile uint32_t PWM1FLTSEN;              /*!< PWM1 Fault Pin Logic Sense                                                   Addresses offset: 0x880 */
  volatile uint32_t PWM1FLTSTAT0;            /*!< PWM1 Fault Status 0                                                          Addresses offset: 0x884 */
  volatile uint32_t PWM1FLTSTAT1;            /*!< PWM1 Fault Status 1                                                          Addresses offset: 0x888 */
  volatile uint32_t RESERVED3[30];
  volatile uint32_t PWM2FLTSTAT0;            /*!< PWM2 Fault Status 0                                                          Addresses offset: 0x904 */
  volatile uint32_t PWM2FLTSTAT1;            /*!< PWM2 Fault Status 1                                                          Addresses offset: 0x908 */
  volatile uint32_t RESERVED4[30];
  volatile uint32_t PWM3FLTSTAT0;            /*!< PWM3 Fault Status 0                                                          Addresses offset: 0x984 */
  volatile uint32_t PWM3FLTSTAT1;            /*!< PWM3 Fault Status 1                                                          Addresses offset: 0x988 */
  volatile uint32_t RESERVED5[397];
  volatile uint32_t PWMPP;                   /*!< PWM Peripheral Properties                                                    Addresses offset: 0xFC0 */

}PWM_RegDef_t;


//*****************************************************************************
//
// PWM Base Addresses Type Casted to PWM_RegDef_t*
//
//*****************************************************************************

#define PWM0                    ((PWM_RegDef_t*) PWM0_BASEADDR)
#define PWM1                    ((PWM_RegDef_t*) PWM1_BASEADDR)

//*****************************************************************************
//
// QEI Base Addresses
//
//*****************************************************************************

#define QEI0_BASEADDR           (APB_BASEADDR + 0x2C000)
#define QEI1_BASEADDR           (APB_BASEADDR + 0x2D000)

//*****************************************************************************
//
// QEI Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t QEICTL;                  /*!< QEI Control                                                                  Addresses offset: 0x000 */
  volatile uint32_t QEISTAT;                 /*!< QEI Status                                                                   Addresses offset: 0x004 */
  volatile uint32_t QEIPOS;                  /*!< QEI Position                                                                 Addresses offset: 0x008 */
  volatile uint32_t QEIMAXPOS;               /*!< QEI Maximum Position                                                         Addresses offset: 0x00C */
  volatile uint32_t QEILOAD;                 /*!< QEI Timer Load                                                               Addresses offset: 0x010 */
  volatile uint32_t QEITIME;                 /*!< QEI Timer                                                                    Addresses offset: 0x014 */
  volatile uint32_t QEICOUNT;                /*!< QEI Velocity Counter                                                         Addresses offset: 0x018 */
  volatile uint32_t QEISPEED;                /*!< QEI Velocity                                                                 Addresses offset: 0x01C */
  volatile uint32_t QEIINTEN;                /*!< QEI Interrupt Enable                                                         Addresses offset: 0x020 */
  volatile uint32_t QEIRIS;                  /*!< QEI Raw Interrupt Status                                                     Addresses offset: 0x024 */
  volatile uint32_t QEIISC;                  /*!< QEI Interrupt Status and Clear                                               Addresses offset: 0x028 */

}QEI_RegDef_t;

//*****************************************************************************
//
// QEI Base Addresses Type Casted to
//
//*****************************************************************************

#define QEI0                    ((QEI_RegDef_t*) QEI0_BASE_ADDR)
#define QEI1                    ((QEI_RegDef_t*) QEI1_BASE_ADDR)

//*****************************************************************************
//
// Timer Base Addresses
//
//*****************************************************************************

#define TIMER0_BASEADDR                  (APB_BASEADDR + 0x30000)
#define TIMER1_BASEADDR                  (APB_BASEADDR + 0x31000)
#define TIMER2_BASEADDR                  (APB_BASEADDR + 0x32000)
#define TIMER3_BASEADDR                  (APB_BASEADDR + 0x33000)
#define TIMER4_BASEADDR                  (APB_BASEADDR + 0x34000)
#define TIMER5_BASEADDR                  (APB_BASEADDR + 0x35000)
#define WTIMER0_BASEADDR                 (APB_BASEADDR + 0x36000)
#define WTIMER1_BASEADDR                 (APB_BASEADDR + 0x37000)
#define WTIMER2_BASEADDR                 (APB_BASEADDR + 0x4C000)
#define WTIMER3_BASEADDR                 (APB_BASEADDR + 0x4D000)
#define WTIMER4_BASEADDR                 (APB_BASEADDR + 0x4E000)
#define WTIMER5_BASEADDR                 (APB_BASEADDR + 0x4F000)

//*****************************************************************************
//
// Timer Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t GPTMCFG;                 /*!< GPTM Configuration                                                           Addresses offset: 0x000 */
  volatile uint32_t GPTMTAMR;                /*!< GPTM Timer A Mode                                                            Addresses offset: 0x004 */
  volatile uint32_t GPTMTBMR;                /*!< GPTM Timer B Mode                                                            Addresses offset: 0x008 */
  volatile uint32_t GPTMCTL;                 /*!< GPTM Control                                                                 Addresses offset: 0x00C */
  volatile uint32_t GPTMSYNC;                /*!< GPTM Synchronize                                                             Addresses offset: 0x010 */
  volatile uint32_t RESERVED0;
  volatile uint32_t GPTMIMR;                 /*!< GPTM Interrupt Mask                                                          Addresses offset: 0x018 */
  volatile uint32_t GPTMRIS;                 /*!< GPTM Raw Interrupt Status                                                    Addresses offset: 0x01C */
  volatile uint32_t GPTMMIS;                 /*!< GPTM Masked Interrupt Status                                                 Addresses offset: 0x020 */
  volatile uint32_t GPTMICR;                 /*!< GPTM Interrupt Clear                                                         Addresses offset: 0x024 */
  volatile uint32_t GPTMTAILR;               /*!< GPTM Timer A Interval Load                                                   Addresses offset: 0x028 */
  volatile uint32_t GPTMTBILR;               /*!< GPTM Timer B Interval Load                                                   Addresses offset: 0x02C */
  volatile uint32_t GPTMTAMATCHR;            /*!< GPTM Timer A Match                                                           Addresses offset: 0x030 */
  volatile uint32_t GPTMTBMATCHR;            /*!< GPTM Timer B Match                                                           Addresses offset: 0x034 */
  volatile uint32_t GPTMTAPR;                /*!< GPTM Timer A Prescale                                                        Addresses offset: 0x038 */
  volatile uint32_t GPTMTBPR;                /*!< GPTM Timer B Prescale                                                        Addresses offset: 0x03C */
  volatile uint32_t GPTMTAPMR;               /*!< GPTM TimerA Prescale Match                                                   Addresses offset: 0x040 */
  volatile uint32_t GPTMTBPMR;               /*!< GPTM TimerB Prescale Match                                                   Addresses offset: 0x044 */
  volatile uint32_t GPTMTAR;                 /*!< GPTM Timer A                                                                 Addresses offset: 0x048 */
  volatile uint32_t GPTMTBR;                 /*!< GPTM Timer B                                                                 Addresses offset: 0x04C */
  volatile uint32_t GPTMTAV;                 /*!< GPTM Timer A Value                                                           Addresses offset: 0x050 */
  volatile uint32_t GPTMTBV;                 /*!< GPTM Timer B Value                                                           Addresses offset: 0x054 */
  volatile uint32_t GPTMRTCPD;               /*!< GPTM RTC Predivide                                                           Addresses offset: 0x058 */
  volatile uint32_t GPTMTAPS;                /*!< GPTM Timer A Prescale Snapshot                                               Addresses offset: 0x05C */
  volatile uint32_t GPTMTBPS;                /*!< GPTM Timer B Prescale Snapshot                                               Addresses offset: 0x060 */
  volatile uint32_t GPTMTAPV;                /*!< GPTM Timer A Prescale Value                                                  Addresses offset: 0x064 */
  volatile uint32_t GPTMTBPV;                /*!< GPTM Timer B Prescale Value                                                  Addresses offset: 0x068 */
  volatile uint32_t RESERVED[981];
  volatile uint32_t GPTMPP;                  /*!< GPTM Peripheral Properties                                                   Addresses offset: 0xFC0 */
}TIMER_RegDef_t;

//*****************************************************************************
//
// Timer Base Addresses Type Casted to Timer_RegDef_t*
//
//*****************************************************************************

#define TIMER0                  ((TIMER_RegDef_t*) TIMER0_BASEADDR)
#define TIMER1                  ((TIMER_RegDef_t*) TIMER1_BASEADDR)
#define TIMER2                  ((TIMER_RegDef_t*) TIMER2_BASEADDR)
#define TIMER3                  ((TIMER_RegDef_t*) TIMER3_BASEADDR)
#define TIMER4                  ((TIMER_RegDef_t*) TIMER4_BASEADDR)
#define TIMER5                  ((TIMER_RegDef_t*) TIMER5_BASEADDR)
#define WTIMER0                 ((TIMER_RegDef_t*) WTIMER0_BASEADDR)
#define WTIMER1                 ((TIMER_RegDef_t*) WTIMER1_BASEADDR)
#define WTIMER2                 ((TIMER_RegDef_t*) WTIMER2_BASEADDR)
#define WTIMER3                 ((TIMER_RegDef_t*) WTIMER3_BASEADDR)
#define WTIMER4                 ((TIMER_RegDef_t*) WTIMER4_BASEADDR)
#define WTIMER5                 ((TIMER_RegDef_t*) WTIMER5_BASEADDR)

//*****************************************************************************
//
// ADC Base Addresses
//
//*****************************************************************************

#define ADC0_BASEADDR           (APB_BASEADDR + 0x38000)
#define ADC1_BASEADDR           (APB_BASEADDR + 0x39000)

//*****************************************************************************
//
// ADC registers (ADC0)
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t ADCACTSS;                /*!< ADC Active Sample Sequencer                                                  Addresses offset: 0x000 */
  volatile uint32_t ADCRIS;                  /*!< ADC Raw Interrupt Status                                                     Addresses offset: 0x004 */
  volatile uint32_t ADCIM;                   /*!< ADC Interrupt Mask                                                           Addresses offset: 0x008 */
  volatile uint32_t ADCISC;                  /*!< ADC Interrupt Status and Clear                                               Addresses offset: 0x00C */
  volatile uint32_t ADCOSTAT;                /*!< ADC Overflow Status                                                          Addresses offset: 0x010 */
  volatile uint32_t ADCEMUX;                 /*!< ADC Event Multiplexer Select                                                 Addresses offset: 0x014 */
  volatile uint32_t ADCUSTAT;                /*!< ADC Underflow Status                                                         Addresses offset: 0x018 */
  volatile uint32_t ADCTSSEL;                /*!< ADC Trigger Source Select                                                    Addresses offset: 0x01C */
  volatile uint32_t ADCSSPRI;                /*!< ADC Sample Sequencer Priority                                                Addresses offset: 0x020 */
  volatile uint32_t ADCSPC;                  /*!< ADC Sample Phase Control                                                     Addresses offset: 0x024 */
  volatile uint32_t ADCPSSI;                 /*!< ADC Processor Sample Sequence Initiate                                       Addresses offset: 0x028 */
  volatile uint32_t RESERVED0;
  volatile uint32_t ADCSAC;                  /*!< ADC Sample Averaging Control                                                 Addresses offset: 0x030 */
  volatile uint32_t ADCDCISC;                /*!<ADC Digital Comparator Interrupt Status and Clear                             Addresses offset: 0x034 */
  volatile uint32_t ADCCTL;                  /*!< ADC Control                                                                  Addresses offset: 0x038 */
  volatile uint32_t RESERVED1;
  volatile uint32_t ADCSSMUX0;               /*!< ADC Sample Sequence Input Multiplexer Select 0                               Addresses offset: 0x040 */
  volatile uint32_t ADCSSCTL0;               /*!< ADC Sample Sequence Control 0                                                Addresses offset: 0x044 */
  volatile uint32_t ADCSSFIFO0;              /*!< ADC Sample Sequence Result FIFO 0                                            Addresses offset: 0x048 */
  volatile uint32_t ADCSSFSTAT0;             /*!< ADC Sample Sequence FIFO 0 Status                                            Addresses offset: 0x04C */
  volatile uint32_t ADCSSOP0;                /*!< ADC Sample Sequence 0 Operation                                              Addresses offset: 0x050 */
  volatile uint32_t ADCSSDC0;                /*!< ADC Sample Sequence 0 Digital Comparator Select                              Addresses offset: 0x054 */
  volatile uint32_t RESERVED2[2];
  volatile uint32_t ADCSSMUX1;               /*!< ADC Sample Sequence Input Multiplexer Select 1                               Addresses offset: 0x060 */
  volatile uint32_t ADCSSCTL1;               /*!< ADC Sample Sequence Control 1                                                Addresses offset: 0x064 */
  volatile uint32_t ADCSSFIFO1;              /*!< ADC Sample Sequence Result FIFO 1                                            Addresses offset: 0x068 */
  volatile uint32_t ADCSSFSTAT1;             /*!< ADC Sample Sequence FIFO 1 Status                                            Addresses offset: 0x06C */
  volatile uint32_t ADCSSOP1;                /*!< ADC Sample Sequence 1 Operation                                              Addresses offset: 0x070 */
  volatile uint32_t ADCSSDC1;                /*!<ADC Sample Sequence 1 Digital Comparator Select                               Addresses offset: 0x074 */
  volatile uint32_t RESERVED3[2];
  volatile uint32_t ADCSSMUX2;               /*!< ADC Sample Sequence Input Multiplexer Select 2                               Addresses offset: 0x080 */
  volatile uint32_t ADCSSCTL2;               /*!< ADC Sample Sequence Control 2                                                Addresses offset: 0x084 */
  volatile uint32_t ADCSSFIFO2;              /*!< ADC Sample Sequence Result FIFO 2                                            Addresses offset: 0x088 */
  volatile uint32_t ADCSSFSTAT2;             /*!< ADC Sample Sequence FIFO 2 Status                                            Addresses offset: 0x08C */
  volatile uint32_t ADCSSOP2;                /*!< ADC Sample Sequence 2 Operation                                              Addresses offset: 0x090 */
  volatile uint32_t ADCSSDC2;                /*!< ADC Sample Sequence 2 Digital Comparator Select                              Addresses offset: 0x094 */
  volatile uint32_t RESERCED4[2];
  volatile uint32_t ADCSSMUX3;               /*!< ADC Sample Sequence Input Multiplexer Select 3                               Addresses offset: 0x0A0 */
  volatile uint32_t ADCSSCTL3;               /*!< ADC Sample Sequence Control 3                                                Addresses offset: 0x0A4 */
  volatile uint32_t ADCSSFIFO3;              /*!< ADC Sample Sequence Result FIFO 3                                            Addresses offset: 0x0A8 */
  volatile uint32_t ADCSSFSTAT3;             /*!< ADC Sample Sequence FIFO 3 Status                                            Addresses offset: 0x0AC */
  volatile uint32_t ADCSSOP3;                /*!< ADC Sample Sequence 3 Operation                                              Addresses offset: 0x0B0 */
  volatile uint32_t ADCSSDC3;                /*!< ADC Sample Sequence 3 Digital Comparator Select                              Addresses offset: 0x0B4 */
  volatile uint32_t RESERVED5[786];
  volatile uint32_t ADCDCRIC;                /*!< ADC Digital Comparator Reset Initial Conditions                              Addresses offset: 0xD00 */
  volatile uint32_t RESERVED6[63];
  volatile uint32_t ADCDCCTL0;               /*!< ADC Digital Comparator Control 0                                             Addresses offset: 0xE00 */
  volatile uint32_t ADCDCCTL1;               /*!< ADC Digital Comparator Control 1                                             Addresses offset: 0xE04 */
  volatile uint32_t ADCDCCTL2;               /*!< ADC Digital Comparator Control 2                                             Addresses offset: 0xE08 */
  volatile uint32_t ADCDCCTL3;               /*!< ADC Digital Comparator Control 3                                             Addresses offset: 0xE0C */
  volatile uint32_t ADCDCCTL4;               /*!< ADC Digital Comparator Control 4                                             Addresses offset: 0xE10 */
  volatile uint32_t ADCDCCTL5;               /*!<ADC Digital Comparator Control 5                                              Addresses offset: 0xE14 */
  volatile uint32_t ADCDCCTL6;               /*!< ADC Digital Comparator Control 6                                             Addresses offset: 0xE18 */
  volatile uint32_t ADCDCCTL7;               /*!<ADC Digital Comparator Control 7                                              Addresses offset: 0xE1C */
  volatile uint32_t RESERVED7[8];
  volatile uint32_t ADCDCCMP0;               /*!< ADC Digital Comparator Range 0                                               Addresses offset: 0xE40 */
  volatile uint32_t ADCDCCMP1;               /*!< ADC Digital Comparator Range 1                                               Addresses offset: 0xE44 */
  volatile uint32_t ADCDCCMP2;               /*!< ADC Digital Comparator Range 2                                               Addresses offset: 0xE48 */
  volatile uint32_t ADCDCCMP3;               /*!< ADC Digital Comparator Range 3                                               Addresses offset: 0xE4C */
  volatile uint32_t ADCDCCMP4;               /*!<ADC Digital Comparator Range 4                                                Addresses offset: 0xE50 */
  volatile uint32_t ADCDCCMP5;               /*!< ADC Digital Comparator Range 5                                               Addresses offset: 0xE54 */
  volatile uint32_t ADCDCCMP6;               /*!< ADC Digital Comparator Range 6                                               Addresses offset: 0xE58 */
  volatile uint32_t ADCDCCMP7;               /*!< ADC Digital Comparator Range 7                                               Addresses offset: 0xE5C */
  volatile uint32_t ADCPP;                   /*!< ADC Peripheral Properties                                                    Addresses offset: 0xFC0 */
  volatile uint32_t ADCPC;                   /*!< ADC Peripheral Configuration                                                 Addresses offset: 0xFC4 */
  volatile uint32_t ADCCC;                   /*!< ADC Clock Configuration                                                      Addresses offset: 0xFC8 */
} ADC_RegDef_t;

//*****************************************************************************
//
// ADC Base Addresses Type Casted to ADC_RegDef_t*
//
//*****************************************************************************

#define ADC0                    ((ADC_RegDef_t*) ADC0_BASEADDR)
#define ADC1                    ((ADC_RegDef_t*) ADC1_BASEADDR)

//*****************************************************************************
//
// Comparator Base Address
//
//*****************************************************************************

#define COMP_BASEADDR           (APB_BASEADDR + 0x3C000)

//*****************************************************************************
//
// Comparator Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t ACMIS;                    /*!< Analog Comparator Masked Interrupt Status                                    Addresses offset: 0x000 */
  volatile uint32_t ACRIS;                    /*!< Analog Comparator Raw Interrupt Status                                       Addresses offset: 0x004 */
  volatile uint32_t ACINTEN;                  /*!< Analog Comparator Interrupt Enable                                           Addresses offset: 0x008 */
  volatile uint32_t RESERVED0;
  volatile uint32_t ACREFCTL;                 /*!< Analog Comparator Reference Voltage Control                                  Addresses offset: 0x010 */
  volatile uint32_t RESERVED1[3];
  volatile uint32_t ACSTAT0;                  /*!< Analog Comparator Status 0                                                   Addresses offset: 0x020 */
  volatile uint32_t ACCTL0;                   /*!< Analog Comparator Control 0                                                  Addresses offset: 0x024 */
  volatile uint32_t RESERVED2[6];
  volatile uint32_t ACSTAT1;                  /*!< Analog Comparator Status 1                                                   Addresses offset: 0x040 */
  volatile uint32_t ACCTL1;                   /*!< Analog Comparator Control 1                                                  Addresses offset: 0x044 */
  volatile uint32_t RESERVED3[990];
  volatile uint32_t ACMPPP;                   /*!< Analog Comparator Peripheral Properties                                      Addresses offset: 0xFC0 */
}COMP_RegDef_t;

//*****************************************************************************
//
// Comparator Base Address Type Casted to COMP_RegDef_t*
//
//*****************************************************************************

#define COMP                    ((COMP_RegDef_t*) COMP_BASEADDR)

//*****************************************************************************
//
// CAN Base Addresses
//
//*****************************************************************************

#define CAN0_BASEADDR           (APB_BASEADDR + 0x40000)
#define CAN1_BASEADDR           (APB_BASEADDR + 0x41000)

//*****************************************************************************
//
// CAN Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t CANCTL;                   /*!< CAN Control                                                                  Addresses offset: 0x000 */
  volatile uint32_t CANSTS;                   /*!< CAN Status                                                                   Addresses offset: 0x004 */
  volatile uint32_t CANERR;                   /*!< CAN Error Counter                                                            Addresses offset: 0x008 */
  volatile uint32_t CANBIT;                   /*!< CAN Bit Timing                                                               Addresses offset: 0x00C */
  volatile uint32_t CANINT;                   /*!< CAN Interrupt                                                                Addresses offset: 0x010 */
  volatile uint32_t CANTST;                   /*!< CAN Test                                                                     Addresses offset: 0x014 */
  volatile uint32_t CANBRPE;                  /*!< CAN Baud Rate Prescaler Extension                                            Addresses offset: 0x018 */
  volatile uint32_t RESERVED0;
  volatile uint32_t CANIF1CRQ;                /*!< CAN IF1 Command Request                                                      Addresses offset: 0x020 */
  volatile uint32_t CANIF1CMSK;               /*!< CAN IF1 Command Mask                                                         Addresses offset: 0x024 */
  volatile uint32_t CANIF1MSK1;               /*!< CAN IF1 Mask 1                                                               Addresses offset: 0x028 */
  volatile uint32_t CANIF1MSK2;               /*!< CAN IF1 Mask 2                                                               Addresses offset: 0x02C */
  volatile uint32_t CANIF1ARB1;               /*!< CAN IF1 Arbitration 1                                                        Addresses offset: 0x030 */
  volatile uint32_t CANIF1ARB2;               /*!< CAN IF1 Arbitration 2                                                        Addresses offset: 0x034 */
  volatile uint32_t CANIF1MCTL;               /*!< CAN IF1 Message Control                                                      Addresses offset: 0x038 */
  volatile uint32_t CANIF1DA1;                /*!< CAN IF1 Data A1                                                              Addresses offset: 0x03C */
  volatile uint32_t CANIF1DA2;                /*!< CAN IF1 Data A2                                                              Addresses offset: 0x040 */
  volatile uint32_t CANIF1DB1;                /*!< CAN IF1 Data B1                                                              Addresses offset: 0x044 */
  volatile uint32_t CANIF1DB2;                /*!< CAN IF1 Data B2                                                              Addresses offset: 0x048 */
  volatile uint32_t RESERVED1[13];
  volatile uint32_t CANIF2CRQ;                /*!< CAN IF2 Command Request                                                      Addresses offset: 0x080 */
  volatile uint32_t CANIF2CMSK;               /*!< CAN IF2 Command Mask                                                         Addresses offset: 0x084 */
  volatile uint32_t CANIF2MSK1;               /*!< CAN IF2 Mask 1                                                               Addresses offset: 0x088 */
  volatile uint32_t CANIF2MSK2;               /*!< CAN IF2 Mask 2                                                               Addresses offset: 0x08C */
  volatile uint32_t CANIF2ARB1;               /*!< CAN IF2 Arbitration 1                                                        Addresses offset: 0x090 */
  volatile uint32_t CANIF2ARB2;               /*!< CAN IF2 Arbitration 2                                                        Addresses offset: 0x094 */
  volatile uint32_t CANIF2MCTL;               /*!< CAN IF2 Message Control                                                      Addresses offset: 0x098 */
  volatile uint32_t CANIF2DA1;                /*!< CAN IF2 Data A1                                                              Addresses offset: 0x09C */
  volatile uint32_t CANIF2DA2;                /*!< CAN IF2 Data A2                                                              Addresses offset: 0x0A0 */
  volatile uint32_t CANIF2DB1;                /*!< CAN IF2 Data B1                                                              Addresses offset: 0x0A4 */
  volatile uint32_t CANIF2DB2;                /*!< CAN IF2 Data B2                                                              Addresses offset: 0x0A8 */
  volatile uint32_t RESERVED2[21];
  volatile uint32_t CANTXRQ1;                 /*!< CAN Transmission Request 1                                                   Addresses offset: 0x100 */
  volatile uint32_t CANTXRQ2;                 /*!< CAN Transmission Request 2                                                   Addresses offset: 0x104 */
  volatile uint32_t RESERVED3[6];
  volatile uint32_t CANNWDA1;                 /*!< CAN New Data 1                                                               Addresses offset: 0x120 */
  volatile uint32_t CANNWDA2;                 /*!< CAN New Data 2                                                               Addresses offset: 0x124 */
  volatile uint32_t RESERVED4[6];
  volatile uint32_t CANMSG1INT;               /*!< CAN Message 1 Interrupt Pending                                              Addresses offset: 0x140 */
  volatile uint32_t CANMSG2INT;               /*!< CAN Message 2 Interrupt Pending                                              Addresses offset: 0x144 */
  volatile uint32_t RESERVED5[6];
  volatile uint32_t CANMSG1VAL;               /*!< CAN Message 1 Valid                                                          Addresses offset: 0x160 */
  volatile uint32_t CANMSG2VAL;               /*!< CAN Message 2 Valid                                                          Addresses offset: 0x164 */
}CAN_RegDef_t;

//*****************************************************************************
//
// CAN Base Addresses Type Casted to CAN_RegDef_t*
//
//*****************************************************************************

#define CAN0                    ((CAN_RegDef_t*) CAN0_BASEADDR)
#define CAN1                    ((CAN_RegDef_t*) CAN1_BASEADDR)

//*****************************************************************************
//
// USB Base Address
//
//*****************************************************************************

#define USB_BASEADDR            (AHB_BASEADDR + 0x00000)

//*****************************************************************************
//
// USB Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint8_t USBFADDR;                  /*!< USB Device Functional Address                                                Addresses offset: 0x000 */
  volatile uint8_t USBPOWER;                  /*!< USB Power                                                                    Addresses offset: 0x001 */
  volatile uint16_t USBTXIS;                  /*!< USB Transmit Interrupt Status                                                Addresses offset: 0x002 */
  volatile uint16_t USBRXIS;                  /*!< USB Receive Interrupt Status                                                 Addresses offset: 0x004 */
  volatile uint16_t USBTXIE;                  /*!< USB Transmit Interrupt Enable                                                Addresses offset: 0x006 */
  volatile uint16_t USBRXIE;                  /*!< USB Receive Interrupt Enable                                                 Addresses offset: 0x008 */
  volatile uint8_t USBIS;                     /*!< USB General Interrupt Status                                                 Addresses offset: 0x00A */
  volatile uint8_t USBIE;                     /*!< USB Interrupt Enable                                                         Addresses offset: 0x00B */
  volatile uint16_t USBFRAME;                 /*!< USB Frame Value                                                              Addresses offset: 0x00C */
  volatile uint8_t USBEPIDX;                  /*!< USB Endpoint Index                                                           Addresses offset: 0x00E */
  volatile uint8_t USBTEST;                   /*!< USB Test Mode                                                                Addresses offset: 0x00F */
  volatile uint32_t RESERVED0[3];
  volatile uint32_t USBFIFO0;                 /*!< USB FIFO Endpoint 0                                                          Addresses offset: 0x020 */
  volatile uint32_t USBFIFO1;                 /*!< USB FIFO Endpoint 1                                                          Addresses offset: 0x024 */
  volatile uint32_t USBFIFO2;                 /*!< USB FIFO Endpoint 2                                                          Addresses offset: 0x028 */
  volatile uint32_t USBFIFO3;                 /*!< USB FIFO Endpoint 3                                                          Addresses offset: 0x02C */
  volatile uint32_t USBFIFO4;                 /*!< USB FIFO Endpoint 4                                                          Addresses offset: 0x030 */
  volatile uint32_t USBFIFO5;                 /*!< USB FIFO Endpoint 5                                                          Addresses offset: 0x034 */
  volatile uint32_t USBFIFO6;                 /*!< USB FIFO Endpoint 6                                                          Addresses offset: 0x038 */
  volatile uint32_t USBFIFO7;                 /*!< USB FIFO Endpoint 7                                                          Addresses offset: 0x03C */
  volatile uint32_t RESERVED1[8];
  volatile uint8_t USBDEVCTL;                 /*!< USB Device Control                                                           Addresses offset: 0x060 */
  volatile uint8_t RESERVED2[1];
  volatile uint8_t USBTXFIFOSZ;               /*!< USB Transmit Dynamic FIFO Sizing                                             Addresses offset: 0x062 */
  volatile uint8_t USBRXFIFOSZ;               /*!< USB Receive Dynamic FIFO Sizing                                              Addresses offset: 0x063 */
  volatile uint16_t USBTXFIFOADD;             /*!< USB Transmit FIFO Start Address                                              Addresses offset: 0x064 */
  volatile uint16_t USBRXFIFOADD;             /*!< USB Receive FIFO Start Address                                               Addresses offset: 0x066 */
  volatile uint32_t RESERVED3[4];
  volatile uint8_t USBCONTIM;                /*!< USB Connect Timing                                                           Addresses offset: 0x07A */
  volatile uint8_t USBVPLEN;                 /*!< USB OTG VBUS Pulse Timing                                                    Addresses offset: 0x07B */
  volatile uint8_t RESERVED4[1];
  volatile uint8_t USBFSEOF;                 /*!< USB Full-Speed Last Transaction to End of Frame Timing                       Addresses offset: 0x07D */
  volatile uint8_t USBLSEOF;                 /*!< USB Low-Speed Last Transaction to End of FrameTiming                         Addresses offset: 0x07E */
  volatile uint8_t RESERVED5[1];
  volatile uint8_t USBTXFUNCADDR0;           /*!< USB Transmit Functional Address Endpoint 0                                   Addresses offset: 0x080 */
  volatile uint8_t RESERVED6[1];
  volatile uint8_t USBTXHUBADDR0;            /*!< USB Transmit Hub Address Endpoint 0                                          Addresses offset: 0x082 */
  volatile uint8_t USBTXHUBPORT0;            /*!< USB Transmit Hub Port Endpoint 0                                             Addresses offset: 0x083 */
  volatile uint8_t RESERVED7[4];
  volatile uint8_t USBTXFUNCADDR1;           /*!< USB Transmit Functional Address Endpoint 1                                   Addresses offset: 0x088 */
  volatile uint8_t RESERVED8[1];
  volatile uint8_t USBTXHUBADDR1;            /*!< USB Transmit Hub Address Endpoint 1                                          Addresses offset: 0x08A */
  volatile uint8_t RESERVED9[1];
  volatile uint8_t USBTXHUBPORT1;            /*!< USB Transmit Hub Port Endpoint 1                                             Addresses offset: 0x08B */
  volatile uint8_t RESERVED10[1];
  volatile uint8_t USBRXFUNCADDR1;           /*!< USB Receive Functional Address Endpoint 1                                    Addresses offset: 0x08C */
  volatile uint8_t RESERVED11[1];
  volatile uint8_t USBRXHUBADDR1;            /*!< USB Receive Hub Address Endpoint 1                                           Addresses offset: 0x08E */
  volatile uint8_t USBRXHUBPORT1;            /*!< USB Receive Hub Port Endpoint 1                                              Addresses offset: 0x08F */
  volatile uint8_t USBTXFUNCADDR2;           /*!< USB Transmit Functional Address Endpoint 2                                   Addresses offset: 0x090 */
  volatile uint8_t RESERVED12[1];
  volatile uint8_t USBTXHUBADDR2;            /*!< USB Transmit Hub Address Endpoint 2                                          Addresses offset: 0x092 */
  volatile uint8_t USBTXHUBPORT2;            /*!< USB Transmit Hub Port Endpoint 2                                             Addresses offset: 0x093 */
  volatile uint8_t USBRXFUNCADDR2;           /*!< USB Receive Functional Address Endpoint 2                                    Addresses offset: 0x094 */
  volatile uint8_t RESERVED13[1];
  volatile uint8_t USBRXHUBADDR2;            /*!< USB Receive Hub Address Endpoint 2                                           Addresses offset: 0x096 */
  volatile uint8_t USBRXHUBPORT2;            /*!< USB Receive Hub Port Endpoint 2                                              Addresses offset: 0x097 */
  volatile uint8_t USBTXFUNCADDR3;           /*!< USB Transmit Functional Address Endpoint 3                                   Addresses offset: 0x098 */
  volatile uint8_t RESERCED14[1];
  volatile uint8_t USBTXHUBADDR3;            /*!< USB Transmit Hub Address Endpoint 3                                          Addresses offset: 0x09A */
  volatile uint8_t USBTXHUBPORT3;            /*!< USB Transmit Hub Port Endpoint 3                                             Addresses offset: 0x09B */
  volatile uint8_t USBRXFUNCADDR3;           /*!< USB Receive Functional Address Endpoint 3                                    Addresses offset: 0x09C */
  volatile uint8_t RESERVED15[1];
  volatile uint8_t USBRXHUBADDR3;            /*!< USB Receive Hub Address Endpoint 3                                           Addresses offset: 0x09E */
  volatile uint8_t USBRXHUBPORT3;            /*!< USB Receive Hub Port Endpoint 3                                              Addresses offset: 0x09F */
  volatile uint8_t USBTXFUNCADDR4;           /*!< USB Transmit Functional Address Endpoint 4                                   Addresses offset: 0x0A0 */
  volatile uint8_t RESERCED16[1];
  volatile uint8_t USBTXHUBADDR4;            /*!< USB Transmit Hub Address Endpoint 4                                          Addresses offset: 0x0A2 */
  volatile uint8_t USBTXHUBPORT4;            /*!< USB Transmit Hub Port Endpoint 4                                             Addresses offset: 0x0A3 */
  volatile uint8_t USBRXFUNCADDR4;           /*!< USB Receive Functional Address Endpoint 4                                    Addresses offset: 0x0A4 */
  volatile uint8_t RESERVED17[1];
  volatile uint8_t USBRXHUBADDR4;            /*!< USB Receive Hub Address Endpoint 4                                           Addresses offset: 0x0A6 */
  volatile uint8_t USBRXHUBPORT4;            /*!< USB Receive Hub Port Endpoint 4                                              Addresses offset: 0x0A7 */
  volatile uint8_t USBTXFUNCADDR5;           /*!< USB Transmit Functional Address Endpoint 5                                   Addresses offset: 0x0A8 */
  volatile uint8_t RESERVED18[1];
  volatile uint8_t USBTXHUBADDR5;            /*!< USB Transmit Hub Address Endpoint 5                                          Addresses offset: 0x0AA */
  volatile uint8_t USBTXHUBPORT5;            /*!< USB Transmit Hub Port Endpoint 5                                             Addresses offset: 0x0AB */
  volatile uint8_t USBRXFUNCADDR5;           /*!< USB Receive Functional Address Endpoint 5                                    Addresses offset: 0x0AC */
  volatile uint8_t RESERVED19[1];
  volatile uint8_t USBRXHUBADDR5;            /*!< USB Receive Hub Address Endpoint 5                                           Addresses offset: 0x0AE */
  volatile uint8_t USBRXHUBPORT5;            /*!< USB Receive Hub Port Endpoint 5                                              Addresses offset: 0x0AF */
  volatile uint8_t USBTXFUNCADDR6;           /*!< USB Transmit Functional Address Endpoint 6                                   Addresses offset: 0x0B0 */
  volatile uint8_t RESERVED20[1];
  volatile uint8_t USBTXHUBADDR6;            /*!< USB Transmit Hub Address Endpoint 6                                          Addresses offset: 0x0B2 */
  volatile uint8_t USBTXHUBPORT6;            /*!< USB Transmit Hub Port Endpoint 6                                             Addresses offset: 0x0B3 */
  volatile uint8_t USBRXFUNCADDR6;           /*!< USB Receive Functional Address Endpoint 6                                    Addresses offset: 0x0B4 */
  volatile uint8_t RESERVED21[1];
  volatile uint8_t USBRXHUBADDR6;            /*!< USB Receive Hub Address Endpoint 6                                           Addresses offset: 0x0B6 */
  volatile uint8_t USBRXHUBPORT6;            /*!< USB Receive Hub Port Endpoint 6                                              Addresses offset: 0x0B7 */
  volatile uint8_t USBTXFUNCADDR7;           /*!< USB Transmit Functional Address Endpoint 7                                   Addresses offset: 0x0B8 */
  volatile uint8_t RESERVED22[1];
  volatile uint8_t USBTXHUBADDR7;            /*!< USB Transmit Hub Address Endpoint 7                                          Addresses offset: 0x0BA */
  volatile uint8_t USBTXHUBPORT7;            /*!< USB Transmit Hub Port Endpoint 7                                             Addresses offset: 0x0BB */
  volatile uint8_t USBRXFUNCADDR7;           /*!< USB Receive Functional Address Endpoint 7                                    Addresses offset: 0x0BC */
  volatile uint8_t RESERVERD23[1];
  volatile uint8_t USBRXHUBADDR7;            /*!< USB Receive Hub Address Endpoint 7                                           Addresses offset: 0x0BE */
  volatile uint8_t USBRXHUBPORT7;            /*!< USB Receive Hub Port Endpoint 7                                              Addresses offset: 0x0BF */
  volatile uint8_t USBCSRL0;                 /*!< USB Control and Status Endpoint 0 Low                                        Addresses offset: 0x102 */
  volatile uint8_t RESERVED24[66];
  volatile uint8_t USBCSRH0;                 /*!< USB Control and Status Endpoint 0 High                                       Addresses offset: 0x103 */
  volatile uint8_t RESERVED25[4];
  volatile uint8_t USBCOUNT0;                /*!< USB Receive Byte Count Endpoint 0                                            Addresses offset: 0x108 */
  volatile uint8_t RESERVED26[1];
  volatile uint8_t USBTYPE0;                 /*!< USB Type Endpoint 0                                                          Addresses offset: 0x10A */
  volatile uint8_t USBNAKLMT;                /*!< USB NAK Limit                                                                Addresses offset: 0x10B */
  volatile uint8_t RESERVED27[4];
  volatile uint16_t USBTXMAXP1;              /*!< USB Maximum Transmit Data Endpoint 1                                         Addresses offset: 0x110 */
  volatile uint8_t USBTXCSRL1;               /*!< USB Transmit Control and Status Endpoint 1 Low                               Addresses offset: 0x112 */
  volatile uint8_t USBTXCSRH1;               /*!< USB Transmit Control and Status Endpoint 1 High                              Addresses offset: 0x113 */
  volatile uint16_t USBRXMAXP1;              /*!< USB Maximum Receive Data Endpoint 1                                          Addresses offset: 0x114 */
  volatile uint8_t USBRXCSRL1;               /*!< USB Receive Control and Status Endpoint 1 Low                                Addresses offset: 0x116 */
  volatile uint8_t USBRXCSRH1;               /*!< USB Receive Control and Status Endpoint 1 High                               Addresses offset: 0x117 */
  volatile uint16_t USBRXCOUNT1;             /*!< USB Receive Byte Count Endpoint 1                                            Addresses offset: 0x118 */
  volatile uint8_t USBTXTYPE1;               /*!< USB Host Transmit Configure Type Endpoint 1                                  Addresses offset: 0x11A */
  volatile uint8_t USBTXINTERVAL1;           /*!< USB Host Transmit Interval Endpoint 1                                        Addresses offset: 0x11B */
  volatile uint8_t USBRXTYPE1;               /*!< USB Host Configure Receive Type Endpoint 1                                   Addresses offset: 0x11C */
  volatile uint8_t USBRXINTERVAL1;           /*!< USB Host Receive Polling Interval Endpoint 1                                 Addresses offset: 0x11D */
  volatile uint8_t RESERVED28[2];
  volatile uint16_t USBTXMAXP2;              /*!< USB Maximum Transmit Data Endpoint 2                                         Addresses offset: 0x120 */
  volatile uint8_t USBTXCSRL2;               /*!< USB Transmit Control and Status Endpoint 2 Low                               Addresses offset: 0x122 */
  volatile uint8_t USBTXCSRH2;               /*!< USB Transmit Control and Status Endpoint 2 High                              Addresses offset: 0x123 */
  volatile uint16_t USBRXMAXP2;              /*!< USB Maximum Receive Data Endpoint 2                                          Addresses offset: 0x124 */
  volatile uint8_t USBRXCSRL2;               /*!< USB Receive Control and Status Endpoint 2 Low                                Addresses offset: 0x126 */
  volatile uint8_t USBRXCSRH2;               /*!< USB Receive Control and Status Endpoint 2 High                               Addresses offset: 0x127 */
  volatile uint16_t USBRXCOUNT2;             /*!< USB Receive Byte Count Endpoint 2                                            Addresses offset: 0x128 */
  volatile uint8_t USBTXTYPE2;               /*!< USB Host Transmit Configure Type Endpoint 2                                  Addresses offset: 0x12A */
  volatile uint8_t USBTXINTERVAL2;           /*!< USB Host Transmit Interval Endpoint 2                                        Addresses offset: 0x12B */
  volatile uint8_t USBRXTYPE2;               /*!< USB Host Configure Receive Type Endpoint 2                                   Addresses offset: 0x12C */
  volatile uint8_t USBRXINTERVAL2;           /*!< USB Host Receive Polling Interval Endpoint 2                                 Addresses offset: 0x12D */
  volatile uint8_t RESERVED29[2];
  volatile uint16_t USBTXMAXP3;              /*!< USB Maximum Transmit Data Endpoint 3                                         Addresses offset: 0x130 */
  volatile uint8_t USBTXCSRL3;               /*!< USB Transmit Control and Status Endpoint 3 Low                               Addresses offset: 0x132 */
  volatile uint8_t USBTXCSRH3;               /*!< USB Transmit Control and Status Endpoint 3 High                              Addresses offset: 0x133 */
  volatile uint16_t USBRXMAXP3;              /*!< USB Maximum Receive Data Endpoint 3                                          Addresses offset: 0x134 */
  volatile uint8_t USBRXCSRL3;               /*!< USB Receive Control and Status Endpoint 3 Low                                Addresses offset: 0x136 */
  volatile uint8_t USBRXCSRH3;               /*!< USB Receive Control and Status Endpoint 3 High                               Addresses offset: 0x137 */
  volatile uint16_t USBRXCOUNT3;             /*!< USB Receive Byte Count Endpoint 3                                            Addresses offset: 0x138 */
  volatile uint8_t USBTXTYPE3;               /*!< USB Host Transmit Configure Type Endpoint 3                                  Addresses offset: 0x13A */
  volatile uint8_t USBTXINTERVAL3;           /*!< USB Host Transmit Interval Endpoint 3                                        Addresses offset: 0x13B */
  volatile uint8_t USBRXTYPE3;               /*!< USB Host Configure Receive Type Endpoint 3                                   Addresses offset: 0x13C */
  volatile uint8_t USBRXINTERVAL3;           /*!< USB Host Receive Polling Interval Endpoint 3                                 Addresses offset: 0x13D */
  volatile uint8_t RESERVED30[2];
  volatile uint16_t USBTXMAXP4;              /*!< USB Maximum Transmit Data Endpoint 4                                         Addresses offset: 0x140 */
  volatile uint8_t USBTXCSRL4;               /*!< USB Transmit Control and Status Endpoint 4 Low                               Addresses offset: 0x142 */
  volatile uint8_t USBTXCSRH4;               /*!< USB Transmit Control and Status Endpoint 4 High                              Addresses offset: 0x143 */
  volatile uint16_t USBRXMAXP4;              /*!< USB Maximum Receive Data Endpoint 4                                          Addresses offset: 0x144 */
  volatile uint8_t USBRXCSRL4;               /*!< USB Receive Control and Status Endpoint 4 Low                                Addresses offset: 0x146 */
  volatile uint8_t USBRXCSRH4;               /*!< USB Receive Control and Status Endpoint 4 High                               Addresses offset: 0x147 */
  volatile uint16_t USBRXCOUNT4;             /*!< USB Receive Byte Count Endpoint 4                                            Addresses offset: 0x148 */
  volatile uint8_t USBTXTYPE4;               /*!< USB Host Transmit Configure Type Endpoint 4                                  Addresses offset: 0x14A */
  volatile uint8_t USBTXINTERVAL4;           /*!< USB Host Transmit Interval Endpoint 4                                        Addresses offset: 0x14B */
  volatile uint8_t USBRXTYPE4;               /*!< USB Host Configure Receive Type Endpoint 4                                   Addresses offset: 0x14C */
  volatile uint8_t USBRXINTERVAL4;           /*!< USB Host Receive Polling Interval Endpoint 4                                 Addresses offset: 0x14D */
  volatile uint8_t RSERVED31[2];
  volatile uint16_t USBTXMAXP5;              /*!< USB Maximum Transmit Data Endpoint 5                                         Addresses offset: 0x150 */
  volatile uint8_t USBTXCSRL5;               /*!< USB Transmit Control and Status Endpoint 5 Low                               Addresses offset: 0x152 */
  volatile uint8_t USBTXCSRH5;               /*!< USB Transmit Control and Status Endpoint 5 High                              Addresses offset: 0x153 */
  volatile uint16_t USBRXMAXP5;              /*!< USB Maximum Receive Data Endpoint 5                                          Addresses offset: 0x154 */
  volatile uint8_t USBRXCSRL5;               /*!< USB Receive Control and Status Endpoint 5 Low                                Addresses offset: 0x156 */
  volatile uint8_t USBRXCSRH5;               /*!< USB Receive Control and Status Endpoint 5 High                               Addresses offset: 0x157 */
  volatile uint16_t USBRXCOUNT5;             /*!< USB Receive Byte Count Endpoint 5                                            Addresses offset: 0x158 */
  volatile uint8_t USBTXTYPE5;               /*!< USB Host Transmit Configure Type Endpoint 5                                  Addresses offset: 0x15A */
  volatile uint8_t USBTXINTERVAL5;           /*!< USB Host Transmit Interval Endpoint 5                                        Addresses offset: 0x15B */
  volatile uint8_t USBRXTYPE5;               /*!< USB Host Configure Receive Type Endpoint 5                                   Addresses offset: 0x15C */
  volatile uint8_t USBRXINTERVAL5;           /*!< USB Host Receive Polling Interval Endpoint 5                                 Addresses offset: 0x15D */
  volatile uint8_t RESERVED32[2];
  volatile uint16_t USBTXMAXP6;              /*!< USB Maximum Transmit Data Endpoint 6                                         Addresses offset: 0x160 */
  volatile uint8_t USBTXCSRL6;               /*!< USB Transmit Control and Status Endpoint 6 Low                               Addresses offset: 0x162 */
  volatile uint8_t USBTXCSRH6;               /*!< USB Transmit Control and Status Endpoint 6 High                              Addresses offset: 0x163 */
  volatile uint16_t USBRXMAXP6;              /*!< USB Maximum Receive Data Endpoint 6                                          Addresses offset: 0x164 */
  volatile uint8_t USBRXCSRL6;               /*!< USB Receive Control and Status Endpoint 6 Low                                Addresses offset: 0x166 */
  volatile uint8_t USBRXCSRH6;               /*!< USB Receive Control and Status Endpoint 6 High                               Addresses offset: 0x167 */
  volatile uint16_t USBRXCOUNT6;             /*!< USB Receive Byte Count Endpoint 6                                            Addresses offset: 0x168 */
  volatile uint8_t USBTXTYPE6;               /*!< USB Host Transmit Configure Type Endpoint 6                                  Addresses offset: 0x16A */
  volatile uint8_t USBTXINTERVAL6;           /*!< USB Host Transmit Interval Endpoint 6                                        Addresses offset: 0x16B */
  volatile uint8_t USBRXTYPE6;               /*!< USB Host Configure Receive Type Endpoint 6                                   Addresses offset: 0x16C */
  volatile uint8_t USBRXINTERVAL6;           /*!< USB Host Receive Polling Interval Endpoint 6                                 Addresses offset: 0x16D */
  volatile uint8_t RESERVED33[2];
  volatile uint16_t USBTXMAXP7;              /*!< USB Maximum Transmit Data Endpoint 7                                         Addresses offset: 0x170 */
  volatile uint8_t USBTXCSRL7;               /*!< USB Transmit Control and Status Endpoint 7 Low                               Addresses offset: 0x172 */
  volatile uint8_t USBTXCSRH7;               /*!< USB Transmit Control and Status Endpoint 7 High                              Addresses offset: 0x173 */
  volatile uint16_t USBRXMAXP7;              /*!< USB Maximum Receive Data Endpoint 7                                          Addresses offset: 0x174 */
  volatile uint8_t USBRXCSRL7;               /*!< USB Receive Control and Status Endpoint 7 Low                                Addresses offset: 0x176 */
  volatile uint8_t USBRXCSRH7;               /*!< USB Receive Control and Status Endpoint 7 High                               Addresses offset: 0x177 */
  volatile uint16_t USBRXCOUNT7;             /*!< USB Receive Byte Count Endpoint 7                                            Addresses offset: 0x178 */
  volatile uint8_t USBTXTYPE7;               /*!< USB Host Transmit Configure Type Endpoint 7                                  Addresses offset: 0x17A */
  volatile uint8_t USBTXINTERVAL7;           /*!< USB Host Transmit Interval Endpoint 7                                        Addresses offset: 0x17B */
  volatile uint8_t USBRXTYPE7;               /*!< USB Host Configure Receive Type Endpoint 7                                   Addresses offset: 0x17C */
  volatile uint8_t USBRXINTERVAL7;           /*!< USB Host Receive Polling Interval Endpoint 7                                 Addresses offset: 0x17D */
  volatile uint8_t RESERVED34[390];
  volatile uint16_t USBRQPKTCOUNT1;          /*!< USB Request Packet Count in Block Transfer Endpoint1                         Addresses offset: 0x304 */
  volatile uint16_t RESERVED35;
  volatile uint16_t USBRQPKTCOUNT2;          /*!< USB Request Packet Count in Block Transfer Endpoint2                         Addresses offset: 0x308 */
  volatile uint16_t RESERVED36;
  volatile uint16_t USBRQPKTCOUNT3;          /*!< USB Request Packet Count in Block Transfer Endpoint3                         Addresses offset: 0x30C */
  volatile uint16_t RESERVED37;
  volatile uint16_t USBRQPKTCOUNT4;          /*!< USB Request Packet Count in Block Transfer Endpoint4                         Addresses offset: 0x310 */
  volatile uint16_t RESERVED38;
  volatile uint16_t USBRQPKTCOUNT5;          /*!< USB Request Packet Count in Block Transfer Endpoint5                         Addresses offset: 0x314 */
  volatile uint16_t RESERVED39;
  volatile uint16_t USBRQPKTCOUNT6;          /*!< USB Request Packet Count in Block Transfer Endpoint6                         Addresses offset: 0x318 */
  volatile uint16_t RESERVED40;
  volatile uint16_t USBRQPKTCOUNT7;          /*!< USB Request Packet Count in Block Transfer Endpoint7                         Addresses offset: 0x31C */
  volatile uint32_t RESERVED41[8];
  volatile uint16_t USBRXDPKTBUFDIS;         /*!< USB Receive Double Packet Buffer Disable                                     Addresses offset: 0x340 */
  volatile uint16_t USBTXDPKTBUFDIS;         /*!< USB Transmit Double Packet Buffer Disable                                    Addresses offset: 0x342 */
  volatile uint32_t RESERVED42[46];
  volatile uint32_t USBEPC;                  /*!< USB External Power Control                                                   Addresses offset: 0x400 */
  volatile uint32_t USBEPCRIS;               /*!< USB External Power Control Raw Interrupt Status                              Addresses offset: 0x404 */
  volatile uint32_t USBEPCIM;                /*!< USB External Power Control Interrupt Mask                                    Addresses offset: 0x408 */
  volatile uint32_t USBEPCISC;               /*!< USB External Power Control Interrupt Status and Clear                        Addresses offset: 0x40C */
  volatile uint32_t USBDRRIS;                /*!< USB Device RESUME Raw Interrupt Status                                       Addresses offset: 0x410 */
  volatile uint32_t USBDRIM;                 /*!< USB Device RESUME Interrupt Mask                                             Addresses offset: 0x414 */
  volatile uint32_t USBDRISC;                /*!< USB Device RESUME Interrupt Status and Clear                                 Addresses offset: 0x418 */
  volatile uint32_t USBGPCS;                 /*!< USB General-Purpose Control and Status                                       Addresses offset: 0x41C */
  volatile uint32_t RESERVED43[4];
  volatile uint32_t USBVDC;                  /*!< USB VBUS Droop Control                                                       Addresses offset: 0x430 */
  volatile uint32_t USBVDCRIS;               /*!< USB VBUS Droop Control Raw Interrupt Status                                  Addresses offset: 0x434 */
  volatile uint32_t USBVDCIM;                /*!< USB VBUS Droop Control Interrupt Mask                                        Addresses offset: 0x438 */
  volatile uint32_t USBVDCISC;               /*!< USB VBUS Droop Control Interrupt Status and Clear                            Addresses offset: 0x43C */
  volatile uint32_t RESERVED44;
  volatile uint32_t USBIDVRIS;               /*!< USB ID Valid Detect Raw Interrupt Status                                     Addresses offset: 0x444 */
  volatile uint32_t USBIDVIM;                /*!< USB ID Valid Detect Interrupt Mask                                           Addresses offset: 0x448 */
  volatile uint32_t USBIDVISC;               /*!< USB ID Valid Detect Interrupt Status and Clear                               Addresses offset: 0x44C */
  volatile uint32_t USBDMASEL;               /*!< USB DMA Select                                                               Addresses offset: 0x450 */
  volatile uint32_t RESERVED45[731];
  volatile uint32_t USBPP;                   /*!< USB Peripheral Properties                                                    Addresses offset: 0xFC0 */
}USB_RegDef_t;

//*****************************************************************************
//
// USB Base Addresses Type Casted to USB_RegDef_t*
//
//*****************************************************************************

#define USB                     ((USB_RegDef_t*) USB_BASEADDR)

//*****************************************************************************
//
// EEPROM Base Address
//
//*****************************************************************************

#define EEPROM_BASEADDR                  (APB_BASEADDR + 0xAF000)

//*****************************************************************************
//
// EEPROM registers (EEPROM)
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t EESIZE;                  /*!< EEPROM Size Information                                                      Addresses offset: 0x000 */
  volatile uint32_t EEBLOCK;                 /*!< EEPROM Current Block                                                         Addresses offset: 0x004 */
  volatile uint32_t EEOFFSET;                /*!< EEPROM Current Offset                                                        Addresses offset: 0x008 */
  volatile uint32_t RESERVED0;
  volatile uint32_t EERDWR;                  /*!< EEPROM Read-Write                                                            Addresses offset: 0x010 */
  volatile uint32_t EERDWRINC;               /*!< EEPROM Read-Write with Increment                                             Addresses offset: 0x014 */
  volatile uint32_t EEDONE;                  /*!< EEPROM Done Status                                                           Addresses offset: 0x018 */
  volatile uint32_t EESUPP;                  /*!< EEPROM Support Control and Status                                            Addresses offset: 0x01C */
  volatile uint32_t EEUNLOCK;                /*!< EEPROM Unlock                                                                Addresses offset: 0x020 */
  volatile uint32_t RSERVED1[3];
  volatile uint32_t EEPROT;                  /*!< EEPROM Protection                                                            Addresses offset: 0x030 */
  volatile uint32_t EEPASS0;                 /*!< EEPROM Password                                                              Addresses offset: 0x034 */
  volatile uint32_t EEPASS1;                 /*!< EEPROM Password                                                              Addresses offset: 0x038 */
  volatile uint32_t EEPASS2;                 /*!< EEPROM Password                                                              Addresses offset: 0x03C */
  volatile uint32_t EEINT;                   /*!< EEPROM Interrupt                                                             Addresses offset: 0x040 */
  volatile uint32_t RESERVED2[3];
  volatile uint32_t EEHIDE;                  /*!< EEPROM Block Hide                                                            Addresses offset: 0x050 */
  volatile uint32_t RESERVED3[11];
  volatile uint32_t EEDBGME;                 /*!< EEPROM Debug Mass Erase                                                      Addresses offset: 0x080 */
  volatile uint32_t RESERVED4[975];
  volatile uint32_t EEPROMPP;                /*!< EEPROM Peripheral Properties                                                 Addresses offset: 0xFC0 */
}EEPROM_RegDef_t;

//*****************************************************************************
//
// EEPROM Base Address Type Casted to EEPROM_RegDef_t*
//
//*****************************************************************************

#define EEPROM                  ((EEPROM_RegDef_t*) EEPROM_BASEADDR)

//*****************************************************************************
//
// System Exception Module Base Address
//
//*****************************************************************************

#define SYSEXC_BASEADDR         (APB_BASEADDR + 0xF9000)

//*****************************************************************************
//
// System Exception Module Structure
//
//*****************************************************************************

typedef struct
{
 volatile uint32_t SYSEXCRIS;               /*!< System Exception Raw Interrupt Status                                        Addresses offset: 0x000 */
 volatile uint32_t SYSEXCIM;                /*!< System Exception Interrupt Mask                                              Addresses offset: 0x004 */
 volatile uint32_t SYSEXCMIS;               /*!< System Exception Masked Interrupt Status                                     Addresses offset: 0x008 */
 volatile uint32_t SYSEXCIC;                /*!< System Exception Interrupt Clear                                             Addresses offset: 0x00C */
}SYSEXC_RegDef_t;

//*****************************************************************************
//
// System Exception Base Address Type Casted to SYSEXC_RegDef_t*
//
//*****************************************************************************

#define SYSEXC                  ((SYSEXC_RegDef_t*) SYSEXC_BASEADDR)

//******************************************************************************
//
// Hibernation Base Address
//
//******************************************************************************

#define HIB_BASEADDR            (APB_BASEADDR + 0xFC000)

//*****************************************************************************
//
// Hibernation module registers Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t HIBRTCC;                 /*!< Hibernation RTC Counter                                                      Addresses offset: 0x000 */
  volatile uint32_t HIBRTCM0;                /*!< Hibernation RTC Match 0                                                      Addresses offset: 0x004 */
  volatile uint32_t RESERVED0;
  volatile uint32_t HIBRTCLD;                /*!< Hibernation RTC Load                                                         Addresses offset: 0x00C */
  volatile uint32_t HIBCTL;                  /*!< Hibernation Control                                                          Addresses offset: 0x010 */
  volatile uint32_t HIBIM;                   /*!< Hibernation Interrupt Mask                                                   Addresses offset: 0x014 */
  volatile uint32_t HIBRIS;                  /*!< Hibernation Raw Interrupt Status                                             Addresses offset: 0x018 */
  volatile uint32_t HIBMIS;                  /*!< Hibernation Masked Interrupt Status                                          Addresses offset: 0x01C */
  volatile uint32_t HIBIC;                   /*!< Hibernation Interrupt Clear                                                  Addresses offset: 0x020 */
  volatile uint32_t HIBRTCT;                 /*!< Hibernation RTC Trim                                                         Addresses offset: 0x024 */
  volatile uint32_t HIBRTCSS;                /*!< Hibernation RTC Sub Seconds                                                  Addresses offset: 0x028 */
  volatile uint32_t RESERVED1;
  volatile uint32_t HIBDATA[14];             /*!< System Exception Interrupt Clear                                             Addresses offset: 0x030-0x06F */
}HIB_RegDef_t;

//*****************************************************************************
//
// Hibernation Base Address Type Casted to HIB_RegDef_t*
//
//*****************************************************************************

#define HIB                     ((HIB_RegDef_t*) HIB_BASEADDR)

//*****************************************************************************
//
// FLASH registers (FLASH CTRL)
//
//*****************************************************************************
#define FLASH_FMA_R             (*((volatile uint32_t *)0x400FD000))
#define FLASH_FMD_R             (*((volatile uint32_t *)0x400FD004))
#define FLASH_FMC_R             (*((volatile uint32_t *)0x400FD008))
#define FLASH_FCRIS_R           (*((volatile uint32_t *)0x400FD00C))
#define FLASH_FCIM_R            (*((volatile uint32_t *)0x400FD010))
#define FLASH_FCMISC_R          (*((volatile uint32_t *)0x400FD014))
#define FLASH_FMC2_R            (*((volatile uint32_t *)0x400FD020))
#define FLASH_FWBVAL_R          (*((volatile uint32_t *)0x400FD030))
#define FLASH_FWBN_R            (*((volatile uint32_t *)0x400FD100))
#define FLASH_FSIZE_R           (*((volatile uint32_t *)0x400FDFC0))
#define FLASH_SSIZE_R           (*((volatile uint32_t *)0x400FDFC4))
#define FLASH_ROMSWMAP_R        (*((volatile uint32_t *)0x400FDFCC))
#define FLASH_RMCTL_R           (*((volatile uint32_t *)0x400FE0F0))
#define FLASH_BOOTCFG_R         (*((volatile uint32_t *)0x400FE1D0))
#define FLASH_USERREG0_R        (*((volatile uint32_t *)0x400FE1E0))
#define FLASH_USERREG1_R        (*((volatile uint32_t *)0x400FE1E4))
#define FLASH_USERREG2_R        (*((volatile uint32_t *)0x400FE1E8))
#define FLASH_USERREG3_R        (*((volatile uint32_t *)0x400FE1EC))
#define FLASH_FMPRE0_R          (*((volatile uint32_t *)0x400FE200))
#define FLASH_FMPRE1_R          (*((volatile uint32_t *)0x400FE204))
#define FLASH_FMPRE2_R          (*((volatile uint32_t *)0x400FE208))
#define FLASH_FMPRE3_R          (*((volatile uint32_t *)0x400FE20C))
#define FLASH_FMPPE0_R          (*((volatile uint32_t *)0x400FE400))
#define FLASH_FMPPE1_R          (*((volatile uint32_t *)0x400FE404))
#define FLASH_FMPPE2_R          (*((volatile uint32_t *)0x400FE408))
#define FLASH_FMPPE3_R          (*((volatile uint32_t *)0x400FE40C))

//*****************************************************************************
//
// System Control Base Address
//
//*****************************************************************************

#define SYSCTL_BASEADDR         (APB_BASEADDR + 0xFE000)                 

//*****************************************************************************
//
// System Control Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t DID0;                    /*!< Device Identification 0                                                      Addresses offset: 0x000 */
  volatile uint32_t DID1;                    /*!< Device Identification 1                                                      Addresses offset: 0x004 */
  volatile uint32_t DC0;                     /*!< Device Capabilities 0                                                        Addresses offset: 0x008 */
  volatile uint32_t RESERVED0;
  volatile uint32_t DC1;                     /*!< Device Capabilities 1                                                        Addresses offset: 0x010 */
  volatile uint32_t DC2;                     /*!< Device Capabilities 2                                                        Addresses offset: 0x014 */
  volatile uint32_t DC3;                     /*!< Device Capabilities 3                                                        Addresses offset: 0x01C */
  volatile uint32_t DC4;                     /*!< Device Capabilities 3                                                        Addresses offset: 0x01C */
  volatile uint32_t DC5;                     /*!< Device Capabilities 5                                                        Addresses offset: 0x020 */
  volatile uint32_t DC6;                     /*!< Device Capabilities 6                                                        Addresses offset: 0x024 */
  volatile uint32_t DC7;                     /*!< Device Capabilities 7                                                        Addresses offset: 0x028 */
  volatile uint32_t DC8;                     /*!< Device Capabilities 8                                                        Addresses offset: 0x02C */
  volatile uint32_t PBORCTL;                 /*!< Brown-Out Reset Control                                                      Addresses offset: 0x030 */
  volatile uint32_t RESERVED1[3];
  volatile uint32_t SRCR0;                   /*!< Software Reset Control 0                                                     Addresses offset: 0x040 */
  volatile uint32_t SRCR1;                   /*!< Software Reset Control 1                                                     Addresses offset: 0x044 */
  volatile uint32_t SRCR2;                   /*!< Software Reset Control 2                                                     Addresses offset: 0x048 */
  volatile uint32_t RESERVED2;
  volatile uint32_t RIS;                     /*!< Raw Interrupt Status                                                         Addresses offset: 0x050 */
  volatile uint32_t IMC;                     /*!< Interrupt Mask Control                                                       Addresses offset: 0x054 */
  volatile uint32_t MISC;                    /*!< Masked Interrupt Status and Clear                                            Addresses offset: 0x058 */
  volatile uint32_t RESC;                    /*!< Reset Cause                                                                  Addresses offset: 0x05C */
  volatile uint32_t RCC;                     /*!< Run-Mode Clock Configuration                                                 Addresses offset: 0x060 */
  volatile uint32_t RESERVED3[2];
  volatile uint32_t GPIOHBCTL;               /*!< GPIO High-Performance Bus Control                                            Addresses offset: 0x06C */
  volatile uint32_t RCC2;                    /*!< Run-Mode Clock Configuration 2                                               Addresses offset: 0x070 */
  volatile uint32_t RESERVED4[2];
  volatile uint32_t MOSCCTL;                 /*!< Main Oscillator Control                                                      Addresses offset: 0x07C */
  volatile uint32_t RESERVED5[32];
  volatile uint32_t RCGC0;                   /*!< Run Mode Clock Gating Control Register 0                                     Addresses offset: 0x100 */
  volatile uint32_t RCGC1;                   /*!< Run Mode Clock Gating Control Register 1                                     Addresses offset: 0x104 */
  volatile uint32_t RCGC2;                   /*!< Run Mode Clock Gating Control Register 2                                     Addresses offset: 0x108 */
  volatile uint32_t RESERVED6;
  volatile uint32_t SCGC0;                   /*!< Sleep Mode Clock Gating Control Register 0                                   Addresses offset: 0x110 */
  volatile uint32_t SCGC1;                   /*!< Sleep Mode Clock Gating Control Register 1                                   Addresses offset: 0x114 */
  volatile uint32_t SCGC2;                   /*!< Sleep Mode Clock Gating Control Register 2                                   Addresses offset: 0x118 */
  volatile uint32_t RESERVED7;
  volatile uint32_t DCGC0;                   /*!< Deep Sleep Mode Clock Gating Control Register 0                              Addresses offset: 0x120 */
  volatile uint32_t DCGC1;                   /*!< Deep-Sleep Mode Clock Gating Control Register 1                              Addresses offset: 0x124 */
  volatile uint32_t DCGC2;                   /*!< Deep Sleep Mode Clock Gating Control Register 2                              Addresses offset: 0x128 */
  volatile uint32_t RESERVED8[6];
  volatile uint32_t DSLPCLKCFG;              /*!< Deep Sleep Clock Configuration                                               Addresses offset: 0x144 */
  volatile uint32_t RESERVED9;
  volatile uint32_t SYSPROP;                 /*!< System Properties                                                            Addresses offset: 0x14C */
  volatile uint32_t PIOSCCAL;                /*!< Precision Internal Oscillator Calibration                                    Addresses offset: 0x150 */
  volatile uint32_t PIOSCSTAT;               /*!< Precision Internal Oscillator Statistics                                     Addresses offset: 0x154 */
  volatile uint32_t RESERVED10[2];
  volatile uint32_t PLLFREQ0;                /*!< PLL Frequency 0                                                              Addresses offset: 0x160 */
  volatile uint32_t PLLFREQ1;                /*!< PLL Frequency 1                                                              Addresses offset: 0x164 */
  volatile uint32_t PLLSTAT;                 /*!< PLL Status                                                                   Addresses offset: 0x168 */
  volatile uint32_t RESERVED11[7];
  volatile uint32_t SLPPWRCFG;               /*!< Sleep Power Configuration                                                    Addresses offset: 0x188 */
  volatile uint32_t DSLPPWRCFG;              /*!< Deep-Sleep Power Configuration                                               Addresses offset: 0x18C */
  volatile uint32_t DC9;                     /*!< Device Capabilities 9                                                        Addresses offset: 0x190 */
  volatile uint32_t RESERVED12[3];
  volatile uint32_t NVMSTAT;                 /*!< Non-Volatile Memory Information                                              Addresses offset: 0x1A0 */
  volatile uint32_t RESERVED13[4];
  volatile uint32_t LDOSPCTL;                /*!< LDO Sleep Power Control                                                      Addresses offset: 0x1B4 */
  volatile uint32_t RESERVED14;
  volatile uint32_t LDODPCTL;                /*!< LDO Sleep Power Calibration                                                  Addresses offset: 0x1B8 */
  volatile uint32_t RESERVED15[80];
  volatile uint32_t PPWD;                    /*!< Watchdog Timer Peripheral Present                                            Addresses offset: 0x300 */
  volatile uint32_t PPTIMER;                 /*!< 16/32-Bit General-Purpose Timer Peripheral Present                           Addresses offset: 0x304 */
  volatile uint32_t PPGPIO;                  /*!< General-Purpose Input/Output Peripheral Present                              Addresses offset: 0x308 */
  volatile uint32_t PPDMA;                   /*!< Micro Direct Memory Access Peripheral Present                                Addresses offset: 0x30C */
  volatile uint32_t RESERVED16;
  volatile uint32_t PPHIB;                   /*!< Hibernation Peripheral Present                                               Addresses offset: 0x314 */
  volatile uint32_t PPUART;                  /*!< Universal Asynchronous Receiver/Transmitter PeripheralPresent                Addresses offset: 0x318 */
  volatile uint32_t PPSSI;                   /*!< Synchronous Serial Interface Peripheral Present                              Addresses offset: 0x31C */
  volatile uint32_t PPI2C;                   /*!< Inter-Integrated Circuit Peripheral Present                                  Addresses offset: 0x320 */
  volatile uint32_t RESERVED17;
  volatile uint32_t PPUSB;                   /*!< Universal Serial Bus Peripheral Present                                      Addresses offset: 0x328 */
  volatile uint32_t RESERVED18[2];
  volatile uint32_t PPCAN;                   /*!< Controller Area Network Peripheral Present                                   Addresses offset: 0x334 */
  volatile uint32_t PPADC;                   /*!< Analog-to-Digital Converter Peripheral Present                               Addresses offset: 0x338 */
  volatile uint32_t PPACMP;                  /*!< Analog Comparator Peripheral Present                                         Addresses offset: 0x33C */
  volatile uint32_t PPPWM;                   /*!< Pulse Width Modulator Peripheral Present                                     Addresses offset: 0x340 */
  volatile uint32_t PPQEI;                   /*!< Quadrature Encoder Interface Peripheral Present                              Addresses offset: 0x344 */
  volatile uint32_t RESERVED19[4];
  volatile uint32_t PPEEPROM;                /*!< EEPROM Peripheral Present                                                    Addresses offset: 0x358 */
  volatile uint32_t PPWTIMER;                /*!< 32/64-Bit Wide General-Purpose Timer PeripheralPresent                       Addresses offset: 0x35C */
  volatile uint32_t RESERVED20[104];
  volatile uint32_t SRWD;                    /*!< Watchdog Timer Software Reset                                                Addresses offset: 0x500 */
  volatile uint32_t SRTIMER;                 /*!< 16/32-Bit General-Purpose Timer Software Reset                               Addresses offset: 0x504 */
  volatile uint32_t SRGPIO;                  /*!< General-Purpose Input/Output Software Reset                                  Addresses offset: 0x508 */
  volatile uint32_t SRDMA;                   /*!< Micro Direct Memory Access Software Reset                                    Addresses offset: 0x50C */
  volatile uint32_t RESERVED21;
  volatile uint32_t SRHIB;                   /*!< Hibernation Software Reset                                                   Addresses offset: 0x514 */
  volatile uint32_t SRUART;                  /*!< Universal Asynchronous Receiver/Transmitter SoftwareReset                    Addresses offset: 0x518 */
  volatile uint32_t SRSSI;                   /*!< Synchronous Serial Interface Software Reset                                  Addresses offset: 0x51C */
  volatile uint32_t SRI2C;                   /*!< Inter-Integrated Circuit Software Reset                                      Addresses offset: 0x520 */
  volatile uint32_t RESERVED22;
  volatile uint32_t SRUSB;                   /*!< Universal Serial Bus Software Reset                                          Addresses offset: 0x528 */
  volatile uint32_t RESREVED23[2];
  volatile uint32_t SRCAN;                   /*!< Controller Area Network Software Reset                                       Addresses offset: 0x534 */
  volatile uint32_t SRADC;                   /*!< Analog-to-Digital Converter Software Reset                                   Addresses offset: 0x538 */
  volatile uint32_t SRACMP;                  /*!< Analog Comparator Software Reset                                             Addresses offset: 0x53C */
  volatile uint32_t SRPWM;                   /*!< Pulse Width Modulator Software Reset                                         Addresses offset: 0x540 */
  volatile uint32_t SRQEI;                   /*!< Quadrature Encoder Interface Software Reset                                  Addresses offset: 0x544 */
  volatile uint32_t RESERVED24[4];
  volatile uint32_t SREEPROM;                /*!< EEPROM Software Reset                                                        Addresses offset: 0x558 */
  volatile uint32_t SRWTIMER;                /*!< 32/64-Bit Wide General-Purpose Timer Software Reset                          Addresses offset: 0x55C */
  volatile uint32_t RESERVED25[40];
  volatile uint32_t RCGCWD;                  /*!< Watchdog Timer Run Mode Clock Gating Control                                 Addresses offset: 0x600 */
  volatile uint32_t RCGCTIMER;               /*!< 16/32-Bit General-Purpose Timer Run Mode Clock GatingControl                 Addresses offset: 0x604 */
  volatile uint32_t RCGCGPIO;                /*!< General-Purpose Input/Output Run Mode Clock GatingControl                    Addresses offset: 0x608 */
  volatile uint32_t RCGCDMA;                 /*!< Micro Direct Memory Access Run Mode Clock GatingControl                      Addresses offset: 0x60C */
  volatile uint32_t RESERVED26;
  volatile uint32_t RCGCHIB;                 /*!< Hibernation Run Mode Clock Gating Control                                    Addresses offset: 0x614 */
  volatile uint32_t RCGCUART;                /*!< Universal Asynchronous Receiver/Transmitter Run ModeClock Gating Control     Addresses offset: 0x618 */
  volatile uint32_t RCGCSSI;                 /*!< Synchronous Serial Interface Run Mode Clock GatingControl                    Addresses offset: 0x61C */
  volatile uint32_t RCGCI2C;                 /*!< Inter-Integrated Circuit Run Mode Clock Gating Control                       Addresses offset: 0x620 */
  volatile uint32_t RESERVED27;
  volatile uint32_t RCGCUSB;                 /*!< Universal Serial Bus Run Mode Clock Gating Control                           Addresses offset: 0x628 */
  volatile uint32_t RESERVED28[2];
  volatile uint32_t RCGCCAN;                 /*!< Controller Area Network Run Mode Clock Gating Control                        Addresses offset: 0x634 */
  volatile uint32_t RCGCADC;                 /*!< Analog-to-Digital Converter Run Mode Clock GatingControl                     Addresses offset: 0x638 */
  volatile uint32_t RCGCACMP;                /*!< Analog Comparator Run Mode Clock Gating Control                              Addresses offset: 0x63C */
  volatile uint32_t RCGCPWM;                 /*!< Pulse Width Modulator Run Mode Clock Gating Control                          Addresses offset: 0x640 */
  volatile uint32_t RCGCQEI;                 /*!< Quadrature Encoder Interface Run Mode Clock GatingControl                    Addresses offset: 0x644 */
  volatile uint32_t RESERVED29[4];
  volatile uint32_t RCGCEEPROM;              /*!< EEPROM Run Mode Clock Gating Control                                         Addresses offset: 0x658 */
  volatile uint32_t RCGCWTIMER;              /*!< 32/64-Bit Wide General-Purpose Timer Run Mode ClockGating Control            Addresses offset: 0x65C */
  volatile uint32_t RESERVED30[40];
  volatile uint32_t SCGCWD;                  /*!< Watchdog Timer Sleep Mode Clock Gating Control                               Addresses offset: 0x700 */
  volatile uint32_t SCGCTIMER;               /*!< 16/32-Bit General-Purpose Timer Sleep Mode ClockGating Control               Addresses offset: 0x704 */
  volatile uint32_t SCGCGPIO;                /*!< General-Purpose Input/Output Sleep Mode Clock GatingControl                  Addresses offset: 0x708 */
  volatile uint32_t SCGCDMA;                 /*!< Micro Direct Memory Access Sleep Mode Clock GatingControl                    Addresses offset: 0x70C */
  volatile uint32_t RESERVED31;
  volatile uint32_t SCGCHIB;                 /*!< Hibernation Sleep Mode Clock Gating Control                                  Addresses offset: 0x714 */
  volatile uint32_t SCGCUART;                /*!< Universal Asynchronous Receiver/Transmitter SleepMode Clock Gating Control   Addresses offset: 0x718 */
  volatile uint32_t SCGCSSI;                 /*!< Synchronous Serial Interface Sleep Mode Clock GatingControl                  Addresses offset: 0x71C */
  volatile uint32_t SCGCI2C;                 /*!< Inter-Integrated Circuit Sleep Mode Clock Gating Control                     Addresses offset: 0x720 */
  volatile uint32_t RESERVED32;
  volatile uint32_t SCGCUSB;                 /*!< Universal Serial Bus Sleep Mode Clock Gating Control                         Addresses offset: 0x728 */
  volatile uint32_t RESERVED33[2];
  volatile uint32_t SCGCCAN;                 /*!< Controller Area Network Sleep Mode Clock GatingControl                       Addresses offset: 0x734 */
  volatile uint32_t SCGCADC;                 /*!< Analog-to-Digital Converter Sleep Mode Clock GatingControl                   Addresses offset: 0x738 */
  volatile uint32_t SCGCACMP;                /*!< Analog Comparator Sleep Mode Clock Gating Control                            Addresses offset: 0x73C */
  volatile uint32_t SCGCPWM;                 /*!< Pulse Width Modulator Sleep Mode Clock Gating Control                        Addresses offset: 0x740 */
  volatile uint32_t SCGCQEI;                 /*!< Quadrature Encoder Interface Sleep Mode Clock GatingControl                  Addresses offset: 0x744 */
  volatile uint32_t RESERVED34[4];
  volatile uint32_t SCGCEEPROM;              /*!< EEPROM Sleep Mode Clock Gating Control                                       Addresses offset: 0x758 */
  volatile uint32_t SCGCWTIMER;              /*!< 32/64-Bit Wide General-Purpose Timer Sleep Mode ClockGating Control          Addresses offset: 0x75C */
  volatile uint32_t RESERVED35[40];
  volatile uint32_t DCGCWD;                  /*!< Watchdog Timer Deep-Sleep Mode Clock Gating Control                          Addresses offset: 0x800 */
  volatile uint32_t DCGCTIMER;               /*!< 16/32-Bit General-Purpose Timer Deep-Sleep ModeClock Gating Control          Addresses offset: 0x804 */
  volatile uint32_t DCGCGPIO;                /*!< General-Purpose Input/Output Deep-Sleep Mode ClockGating Control             Addresses offset: 0x808 */
  volatile uint32_t DCGCDMA;                 /*!< Micro Direct Memory Access Deep-Sleep Mode ClockGating Control               Addresses offset: 0x80C */
  volatile uint32_t RESERVED36;
  volatile uint32_t DCGCHIB;                 /*!< Hibernation Deep-Sleep Mode Clock Gating Control                             Addresses offset: 0x814 */
  volatile uint32_t DCGCUART;             /*!< Universal Asynchronous Receiver/TransmitterDeep-Sleep Mode Clock Gating Control Addresses offset: 0x818 */
  volatile uint32_t DCGCSSI;                 /*!< Synchronous Serial Interface Deep-Sleep Mode ClockGating Control             Addresses offset: 0x81C */
  volatile uint32_t DCGCI2C;                 /*!< Inter-Integrated Circuit Deep-Sleep Mode Clock GatingControl                 Addresses offset: 0x820 */
  volatile uint32_t RESERVED37;
  volatile uint32_t DCGCUSB;                 /*!< Universal Serial Bus Deep-Sleep Mode Clock GatingControl                     Addresses offset: 0x828 */
  volatile uint32_t RESERVED38[2];
  volatile uint32_t DCGCCAN;                 /*!< Controller Area Network Deep-Sleep Mode Clock GatingControl                  Addresses offset: 0x834 */
  volatile uint32_t DCGCADC;                 /*!< Analog-to-Digital Converter Deep-Sleep Mode ClockGating Control              Addresses offset: 0x838 */
  volatile uint32_t DCGCACMP;                /*!< Analog Comparator Deep-Sleep Mode Clock GatingControl                        Addresses offset: 0x83C */
  volatile uint32_t DCGCPWM;                 /*!< Pulse Width Modulator Deep-Sleep Mode Clock GatingControl                    Addresses offset: 0x840 */
  volatile uint32_t DCGCQEI;                 /*!< Quadrature Encoder Interface Deep-Sleep Mode ClockGating Control             Addresses offset: 0x844 */
  volatile uint32_t RESERVED39[4];
  volatile uint32_t DCGCEEPROM;              /*!< EEPROM Deep-Sleep Mode Clock Gating Control                                  Addresses offset: 0x858 */
  volatile uint32_t DCGCWTIMER;              /*!< 32/64-Bit Wide General-Purpose Timer Deep-Sleep ModeClock Gating Control     Addresses offset: 0x85C */
  volatile uint32_t RESERVED40[104];
  volatile uint32_t PRWD;                    /*!< Watchdog Timer Peripheral Ready                                              Addresses offset: 0xA00 */
  volatile uint32_t PRTIMER;                 /*!< 16/32-Bit General-Purpose Timer Peripheral Ready                             Addresses offset: 0xA04 */
  volatile uint32_t PRGPIO;                  /*!< General-Purpose Input/Output Peripheral Ready                                Addresses offset: 0xA08 */
  volatile uint32_t PRDMA;                   /*!< Micro Direct Memory Access Peripheral Ready                                  Addresses offset: 0xA0C */
  volatile uint32_t RESERVED41;
  volatile uint32_t PRHIB;                   /*!< Hibernation Peripheral Ready                                                 Addresses offset: 0xA14 */
  volatile uint32_t PRUART;                  /*!< Universal Asynchronous Receiver/Transmitter PeripheralReady                  Addresses offset: 0xA18 */
  volatile uint32_t PRSSI;                   /*!< Synchronous Serial Interface Peripheral Ready                                Addresses offset: 0xA1C */
  volatile uint32_t PRI2C;                   /*!< Inter-Integrated Circuit Peripheral Ready                                    Addresses offset: 0xA20 */
  volatile uint32_t RESERVED42;
  volatile uint32_t PRUSB;                   /*!< Universal Serial Bus Peripheral Ready                                        Addresses offset: 0xA28 */
  volatile uint32_t RESERVED43[2];
  volatile uint32_t PRCAN;                   /*!< Controller Area Network Peripheral Ready                                     Addresses offset: 0xA34 */
  volatile uint32_t PRADC;                   /*!< Analog-to-Digital Converter Peripheral Ready                                 Addresses offset: 0xA38 */
  volatile uint32_t PRACMP;                  /*!< Analog Comparator Peripheral Ready                                           Addresses offset: 0xA3C */
  volatile uint32_t PRPWM;                   /*!< Pulse Width Modulator Peripheral Ready                                       Addresses offset: 0xA40 */
  volatile uint32_t PRQEI;                   /*!< Quadrature Encoder Interface Peripheral Ready                                Addresses offset: 0xA44 */
  volatile uint32_t RESERVED44[4];
  volatile uint32_t PREEPROM;                /*!< EEPROM Peripheral Ready                                                      Addresses offset: 0xA58 */
  volatile uint32_t PRWTIMER;                /*!< 32/64-Bit Wide General-Purpose Timer Peripheral Ready                        Addresses offset: 0xA5C */
}SYSCTL_RegDef_t;

//*****************************************************************************
//
// System Control Base Address Type Casted to SYSCTL_RegDef_t*
//
//*****************************************************************************

#define SYSCTL                  ((SYSCTL_RegDef_t*) SYSCTL_BASEADDR)

//*****************************************************************************
//
// Micro Direct Memory Access Base Address
//
//*****************************************************************************

#define UDMA_BASEADDR           (APB_BASEADDR + 0x100000)

//*****************************************************************************
//
// Micro Direct Memory Access Structure
//
//*****************************************************************************

typedef struct
{
  volatile uint32_t DMASTAT;                 /*!< DMA Status                                                                   Addresses offset: 0x000 */
  volatile uint32_t DMACFG;                  /*!< DMA Configuration                                                            Addresses offset: 0x004 */
  volatile uint32_t DMACTLBASE;              /*!< DMA Channel Control Base Pointer                                             Addresses offset: 0x008 */
  volatile uint32_t DMAALTBASE;              /*!< DMA Alternate Channel Control Base Pointer                                   Addresses offset: 0x00C */
  volatile uint32_t DMAWAITSTAT;             /*!< DMA Channel Wait-on-Request Status                                           Addresses offset: 0x010 */
  volatile uint32_t DMASWREQ;                /*!< DMA Channel Software Request                                                 Addresses offset: 0x014 */
  volatile uint32_t DMAUSEBURSTSET;          /*!< DMA Channel Useburst Set                                                     Addresses offset: 0x018 */
  volatile uint32_t DMAUSEBURSTCLR;          /*!< DMA Channel Useburst Clear                                                   Addresses offset: 0x01C */
  volatile uint32_t DMAREQMASKSET;           /*!< DMA Channel Request Mask Set                                                 Addresses offset: 0x020 */
  volatile uint32_t DMAREQMASKCLR;           /*!< DMA Channel Request Mask Clear                                               Addresses offset: 0x024 */
  volatile uint32_t DMAENASET;               /*!< DMA Channel Enable Set                                                       Addresses offset: 0x028 */
  volatile uint32_t DMAENACLR;               /*!< DMA Channel Enable Clear                                                     Addresses offset: 0x02C */
  volatile uint32_t DMAALTSET;               /*!< DMA Channel Primary Alternate Set                                            Addresses offset: 0x030 */
  volatile uint32_t DMAALTCLR;               /*!< DMA Channel Primary Alternate Clear                                          Addresses offset: 0x034 */
  volatile uint32_t DMAPRIOSET;              /*!< DMA Channel Priority Set                                                     Addresses offset: 0x038 */
  volatile uint32_t DMAPRIOCLR;              /*!< DMA Channel Priority Clear                                                   Addresses offset: 0x03C */
  volatile uint32_t RESERVERD0[3];
  volatile uint32_t DMAERRCLR;               /*!< DMA Bus Error Clear                                                          Addresses offset: 0x04C */
  volatile uint32_t RESERVED1[300];
  volatile uint32_t DMACHASGN;               /*!< DMA Channel Assignment                                                       Addresses offset: 0x500 */
  volatile uint32_t DMACHIS;                 /*!< DMA Channel Interrupt Status                                                 Addresses offset: 0x504 */
  volatile uint32_t RESERVED2;
  volatile uint32_t DMACHMAP0;               /*!< DMA Channel Map Select 0                                                     Addresses offset: 0x510 */
  volatile uint32_t DMACHMAP1;               /*!< DMA Channel Map Select 1                                                     Addresses offset: 0x514 */
  volatile uint32_t DMACHMAP2;               /*!< DMA Channel Map Select 2                                                     Addresses offset: 0x518 */
  volatile uint32_t DMACHMAP3;               /*!< DMA Channel Map Select 3                                                     Addresses offset: 0x51C */
}UDMA_RegDef_t;

//*****************************************************************************
//
// Micro Direct Memory Access Base Address Type Casted to UDMA_RegDef_t*
//
//*****************************************************************************

#define UDMA                   ((UDMA_RegDef_t*) UDMA_BASEADDR)

//*****************************************************************************
//
// Micro Direct Memory Access (uDMA) offsets (UDMA)
//
//*****************************************************************************
#define UDMA_SRCENDP            0x00000000  // DMA Channel Source Address End
                                            // Pointer
#define UDMA_DSTENDP            0x00000004  // DMA Channel Destination Address
                                            // End Pointer
#define UDMA_CHCTL              0x00000008  // DMA Channel Control Word

//*****************************************************************************
//
// NVIC registers (NVIC)
//
//*****************************************************************************
#define NVIC_ACTLR_R            (*((volatile uint32_t *)0xE000E008))
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_EN0_R              (*((volatile uint32_t *)0xE000E100))
#define NVIC_EN1_R              (*((volatile uint32_t *)0xE000E104))
#define NVIC_EN2_R              (*((volatile uint32_t *)0xE000E108))
#define NVIC_EN3_R              (*((volatile uint32_t *)0xE000E10C))
#define NVIC_EN4_R              (*((volatile uint32_t *)0xE000E110))
#define NVIC_DIS0_R             (*((volatile uint32_t *)0xE000E180))
#define NVIC_DIS1_R             (*((volatile uint32_t *)0xE000E184))
#define NVIC_DIS2_R             (*((volatile uint32_t *)0xE000E188))
#define NVIC_DIS3_R             (*((volatile uint32_t *)0xE000E18C))
#define NVIC_DIS4_R             (*((volatile uint32_t *)0xE000E190))
#define NVIC_PEND0_R            (*((volatile uint32_t *)0xE000E200))
#define NVIC_PEND1_R            (*((volatile uint32_t *)0xE000E204))
#define NVIC_PEND2_R            (*((volatile uint32_t *)0xE000E208))
#define NVIC_PEND3_R            (*((volatile uint32_t *)0xE000E20C))
#define NVIC_PEND4_R            (*((volatile uint32_t *)0xE000E210))
#define NVIC_UNPEND0_R          (*((volatile uint32_t *)0xE000E280))
#define NVIC_UNPEND1_R          (*((volatile uint32_t *)0xE000E284))
#define NVIC_UNPEND2_R          (*((volatile uint32_t *)0xE000E288))
#define NVIC_UNPEND3_R          (*((volatile uint32_t *)0xE000E28C))
#define NVIC_UNPEND4_R          (*((volatile uint32_t *)0xE000E290))
#define NVIC_ACTIVE0_R          (*((volatile uint32_t *)0xE000E300))
#define NVIC_ACTIVE1_R          (*((volatile uint32_t *)0xE000E304))
#define NVIC_ACTIVE2_R          (*((volatile uint32_t *)0xE000E308))
#define NVIC_ACTIVE3_R          (*((volatile uint32_t *)0xE000E30C))
#define NVIC_ACTIVE4_R          (*((volatile uint32_t *)0xE000E310))
#define NVIC_PRI0_R             (*((volatile uint32_t *)0xE000E400))
#define NVIC_PRI1_R             (*((volatile uint32_t *)0xE000E404))
#define NVIC_PRI2_R             (*((volatile uint32_t *)0xE000E408))
#define NVIC_PRI3_R             (*((volatile uint32_t *)0xE000E40C))
#define NVIC_PRI4_R             (*((volatile uint32_t *)0xE000E410))
#define NVIC_PRI5_R             (*((volatile uint32_t *)0xE000E414))
#define NVIC_PRI6_R             (*((volatile uint32_t *)0xE000E418))
#define NVIC_PRI7_R             (*((volatile uint32_t *)0xE000E41C))
#define NVIC_PRI8_R             (*((volatile uint32_t *)0xE000E420))
#define NVIC_PRI9_R             (*((volatile uint32_t *)0xE000E424))
#define NVIC_PRI10_R            (*((volatile uint32_t *)0xE000E428))
#define NVIC_PRI11_R            (*((volatile uint32_t *)0xE000E42C))
#define NVIC_PRI12_R            (*((volatile uint32_t *)0xE000E430))
#define NVIC_PRI13_R            (*((volatile uint32_t *)0xE000E434))
#define NVIC_PRI14_R            (*((volatile uint32_t *)0xE000E438))
#define NVIC_PRI15_R            (*((volatile uint32_t *)0xE000E43C))
#define NVIC_PRI16_R            (*((volatile uint32_t *)0xE000E440))
#define NVIC_PRI17_R            (*((volatile uint32_t *)0xE000E444))
#define NVIC_PRI18_R            (*((volatile uint32_t *)0xE000E448))
#define NVIC_PRI19_R            (*((volatile uint32_t *)0xE000E44C))
#define NVIC_PRI20_R            (*((volatile uint32_t *)0xE000E450))
#define NVIC_PRI21_R            (*((volatile uint32_t *)0xE000E454))
#define NVIC_PRI22_R            (*((volatile uint32_t *)0xE000E458))
#define NVIC_PRI23_R            (*((volatile uint32_t *)0xE000E45C))
#define NVIC_PRI24_R            (*((volatile uint32_t *)0xE000E460))
#define NVIC_PRI25_R            (*((volatile uint32_t *)0xE000E464))
#define NVIC_PRI26_R            (*((volatile uint32_t *)0xE000E468))
#define NVIC_PRI27_R            (*((volatile uint32_t *)0xE000E46C))
#define NVIC_PRI28_R            (*((volatile uint32_t *)0xE000E470))
#define NVIC_PRI29_R            (*((volatile uint32_t *)0xE000E474))
#define NVIC_PRI30_R            (*((volatile uint32_t *)0xE000E478))
#define NVIC_PRI31_R            (*((volatile uint32_t *)0xE000E47C))
#define NVIC_PRI32_R            (*((volatile uint32_t *)0xE000E480))
#define NVIC_PRI33_R            (*((volatile uint32_t *)0xE000E484))
#define NVIC_PRI34_R            (*((volatile uint32_t *)0xE000E488))
#define NVIC_CPUID_R            (*((volatile uint32_t *)0xE000ED00))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_VTABLE_R           (*((volatile uint32_t *)0xE000ED08))
#define NVIC_APINT_R            (*((volatile uint32_t *)0xE000ED0C))
#define NVIC_SYS_CTRL_R         (*((volatile uint32_t *)0xE000ED10))
#define NVIC_CFG_CTRL_R         (*((volatile uint32_t *)0xE000ED14))
#define NVIC_SYS_PRI1_R         (*((volatile uint32_t *)0xE000ED18))
#define NVIC_SYS_PRI2_R         (*((volatile uint32_t *)0xE000ED1C))
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))
#define NVIC_SYS_HND_CTRL_R     (*((volatile uint32_t *)0xE000ED24))
#define NVIC_FAULT_STAT_R       (*((volatile uint32_t *)0xE000ED28))
#define NVIC_HFAULT_STAT_R      (*((volatile uint32_t *)0xE000ED2C))
#define NVIC_DEBUG_STAT_R       (*((volatile uint32_t *)0xE000ED30))
#define NVIC_MM_ADDR_R          (*((volatile uint32_t *)0xE000ED34))
#define NVIC_FAULT_ADDR_R       (*((volatile uint32_t *)0xE000ED38))
#define NVIC_CPAC_R             (*((volatile uint32_t *)0xE000ED88))
#define NVIC_MPU_TYPE_R         (*((volatile uint32_t *)0xE000ED90))
#define NVIC_MPU_CTRL_R         (*((volatile uint32_t *)0xE000ED94))
#define NVIC_MPU_NUMBER_R       (*((volatile uint32_t *)0xE000ED98))
#define NVIC_MPU_BASE_R         (*((volatile uint32_t *)0xE000ED9C))
#define NVIC_MPU_ATTR_R         (*((volatile uint32_t *)0xE000EDA0))
#define NVIC_MPU_BASE1_R        (*((volatile uint32_t *)0xE000EDA4))
#define NVIC_MPU_ATTR1_R        (*((volatile uint32_t *)0xE000EDA8))
#define NVIC_MPU_BASE2_R        (*((volatile uint32_t *)0xE000EDAC))
#define NVIC_MPU_ATTR2_R        (*((volatile uint32_t *)0xE000EDB0))
#define NVIC_MPU_BASE3_R        (*((volatile uint32_t *)0xE000EDB4))
#define NVIC_MPU_ATTR3_R        (*((volatile uint32_t *)0xE000EDB8))
#define NVIC_DBG_CTRL_R         (*((volatile uint32_t *)0xE000EDF0))
#define NVIC_DBG_XFER_R         (*((volatile uint32_t *)0xE000EDF4))
#define NVIC_DBG_DATA_R         (*((volatile uint32_t *)0xE000EDF8))
#define NVIC_DBG_INT_R          (*((volatile uint32_t *)0xE000EDFC))
#define NVIC_SW_TRIG_R          (*((volatile uint32_t *)0xE000EF00))
#define NVIC_FPCC_R             (*((volatile uint32_t *)0xE000EF34))
#define NVIC_FPCA_R             (*((volatile uint32_t *)0xE000EF38))
#define NVIC_FPDSC_R            (*((volatile uint32_t *)0xE000EF3C))

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOAD register.
//
//*****************************************************************************
#define WDT_LOAD_M              0xFFFFFFFF  // Watchdog Load Value
#define WDT_LOAD_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_VALUE register.
//
//*****************************************************************************
#define WDT_VALUE_M             0xFFFFFFFF  // Watchdog Value
#define WDT_VALUE_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_CTL register.
//
//*****************************************************************************
#define WDT_CTL_WRC             0x80000000  // Write Complete
#define WDT_CTL_INTTYPE         0x00000004  // Watchdog Interrupt Type
#define WDT_CTL_RESEN           0x00000002  // Watchdog Reset Enable
#define WDT_CTL_INTEN           0x00000001  // Watchdog Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_ICR register.
//
//*****************************************************************************
#define WDT_ICR_M               0xFFFFFFFF  // Watchdog Interrupt Clear
#define WDT_ICR_S               0

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_RIS register.
//
//*****************************************************************************
#define WDT_RIS_WDTRIS          0x00000001  // Watchdog Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_MIS register.
//
//*****************************************************************************
#define WDT_MIS_WDTMIS          0x00000001  // Watchdog Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_TEST register.
//
//*****************************************************************************
#define WDT_TEST_STALL          0x00000100  // Watchdog Stall Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOCK register.
//
//*****************************************************************************
#define WDT_LOCK_M              0xFFFFFFFF  // Watchdog Lock
#define WDT_LOCK_UNLOCKED       0x00000000  // Unlocked
#define WDT_LOCK_LOCKED         0x00000001  // Locked
#define WDT_LOCK_UNLOCK         0x1ACCE551  // Unlocks the watchdog timer








//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
//*****************************************************************************
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_FRF_TI          0x00000010  // Synchronous Serial Frame Format
#define SSI_CR0_FRF_NMW         0x00000020  // MICROWIRE Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select
#define SSI_CR0_DSS_4           0x00000003  // 4-bit data
#define SSI_CR0_DSS_5           0x00000004  // 5-bit data
#define SSI_CR0_DSS_6           0x00000005  // 6-bit data
#define SSI_CR0_DSS_7           0x00000006  // 7-bit data
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR0_DSS_9           0x00000008  // 9-bit data
#define SSI_CR0_DSS_10          0x00000009  // 10-bit data
#define SSI_CR0_DSS_11          0x0000000A  // 11-bit data
#define SSI_CR0_DSS_12          0x0000000B  // 12-bit data
#define SSI_CR0_DSS_13          0x0000000C  // 13-bit data
#define SSI_CR0_DSS_14          0x0000000D  // 14-bit data
#define SSI_CR0_DSS_15          0x0000000E  // 15-bit data
#define SSI_CR0_DSS_16          0x0000000F  // 16-bit data
#define SSI_CR0_SCR_S           8

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
//*****************************************************************************
#define SSI_CR1_EOT             0x00000010  // End of Transmission
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
                                            // Enable
#define SSI_CR1_LBM             0x00000001  // SSI Loopback Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
//*****************************************************************************
#define SSI_DR_DATA_M           0x0000FFFF  // SSI Receive/Transmit Data
#define SSI_DR_DATA_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
//*****************************************************************************
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit
#define SSI_SR_RFF              0x00000008  // SSI Receive FIFO Full
#define SSI_SR_RNE              0x00000004  // SSI Receive FIFO Not Empty
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full
#define SSI_SR_TFE              0x00000001  // SSI Transmit FIFO Empty

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
//*****************************************************************************
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor
#define SSI_CPSR_CPSDVSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
//*****************************************************************************
#define SSI_IM_TXIM             0x00000008  // SSI Transmit FIFO Interrupt Mask
#define SSI_IM_RXIM             0x00000004  // SSI Receive FIFO Interrupt Mask
#define SSI_IM_RTIM             0x00000002  // SSI Receive Time-Out Interrupt
                                            // Mask
#define SSI_IM_RORIM            0x00000001  // SSI Receive Overrun Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
#define SSI_RIS_TXRIS           0x00000008  // SSI Transmit FIFO Raw Interrupt
                                            // Status
#define SSI_RIS_RXRIS           0x00000004  // SSI Receive FIFO Raw Interrupt
                                            // Status
#define SSI_RIS_RTRIS           0x00000002  // SSI Receive Time-Out Raw
                                            // Interrupt Status
#define SSI_RIS_RORRIS          0x00000001  // SSI Receive Overrun Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
#define SSI_MIS_TXMIS           0x00000008  // SSI Transmit FIFO Masked
                                            // Interrupt Status
#define SSI_MIS_RXMIS           0x00000004  // SSI Receive FIFO Masked
                                            // Interrupt Status
#define SSI_MIS_RTMIS           0x00000002  // SSI Receive Time-Out Masked
                                            // Interrupt Status
#define SSI_MIS_RORMIS          0x00000001  // SSI Receive Overrun Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
#define SSI_ICR_RTIC            0x00000002  // SSI Receive Time-Out Interrupt
                                            // Clear
#define SSI_ICR_RORIC           0x00000001  // SSI Receive Overrun Interrupt
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DMACTL register.
//
//*****************************************************************************
#define SSI_DMACTL_TXDMAE       0x00000002  // Transmit DMA Enable
#define SSI_DMACTL_RXDMAE       0x00000001  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CC register.
//
//*****************************************************************************
#define SSI_CC_CS_M             0x0000000F  // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL        0x00000000  // System clock (based on clock
                                            // source and divisor factor)
#define SSI_CC_CS_PIOSC         0x00000005  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DR register.
//
//*****************************************************************************
#define UART_DR_OE              0x00000800  // UART Overrun Error
#define UART_DR_BE              0x00000400  // UART Break Error
#define UART_DR_PE              0x00000200  // UART Parity Error
#define UART_DR_FE              0x00000100  // UART Framing Error
#define UART_DR_DATA_M          0x000000FF  // Data Transmitted or Received
#define UART_DR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RSR register.
//
//*****************************************************************************
#define UART_RSR_OE             0x00000008  // UART Overrun Error
#define UART_RSR_BE             0x00000004  // UART Break Error
#define UART_RSR_PE             0x00000002  // UART Parity Error
#define UART_RSR_FE             0x00000001  // UART Framing Error

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
//*****************************************************************************
#define UART_ECR_DATA_M         0x000000FF  // Error Clear
#define UART_ECR_DATA_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FR register.
//
//*****************************************************************************
#define UART_FR_TXFE            0x00000080  // UART Transmit FIFO Empty
#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_FR_BUSY            0x00000008  // UART Busy
#define UART_FR_CTS             0x00000001  // Clear To Send

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
//*****************************************************************************
#define UART_ILPR_ILPDVSR_M     0x000000FF  // IrDA Low-Power Divisor
#define UART_ILPR_ILPDVSR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IBRD register.
//
//*****************************************************************************
#define UART_IBRD_DIVINT_M      0x0000FFFF  // Integer Baud-Rate Divisor
#define UART_IBRD_DIVINT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FBRD register.
//
//*****************************************************************************
#define UART_FBRD_DIVFRAC_M     0x0000003F  // Fractional Baud-Rate Divisor
#define UART_FBRD_DIVFRAC_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
//*****************************************************************************
#define UART_LCRH_SPS           0x00000080  // UART Stick Parity Select
#define UART_LCRH_WLEN_M        0x00000060  // UART Word Length
#define UART_LCRH_WLEN_5        0x00000000  // 5 bits (default)
#define UART_LCRH_WLEN_6        0x00000020  // 6 bits
#define UART_LCRH_WLEN_7        0x00000040  // 7 bits
#define UART_LCRH_WLEN_8        0x00000060  // 8 bits
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_LCRH_STP2          0x00000008  // UART Two Stop Bits Select
#define UART_LCRH_EPS           0x00000004  // UART Even Parity Select
#define UART_LCRH_PEN           0x00000002  // UART Parity Enable
#define UART_LCRH_BRK           0x00000001  // UART Send Break

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CTL register.
//
//*****************************************************************************
#define UART_CTL_CTSEN          0x00008000  // Enable Clear To Send
#define UART_CTL_RTSEN          0x00004000  // Enable Request to Send
#define UART_CTL_RTS            0x00000800  // Request to Send
#define UART_CTL_RXE            0x00000200  // UART Receive Enable
#define UART_CTL_TXE            0x00000100  // UART Transmit Enable
#define UART_CTL_LBE            0x00000080  // UART Loop Back Enable
#define UART_CTL_HSE            0x00000020  // High-Speed Enable
#define UART_CTL_EOT            0x00000010  // End of Transmission
#define UART_CTL_SMART          0x00000008  // ISO 7816 Smart Card Support
#define UART_CTL_SIRLP          0x00000004  // UART SIR Low-Power Mode
#define UART_CTL_SIREN          0x00000002  // UART SIR Enable
#define UART_CTL_UARTEN         0x00000001  // UART Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IFLS register.
//
//*****************************************************************************
#define UART_IFLS_RX_M          0x00000038  // UART Receive Interrupt FIFO
                                            // Level Select
#define UART_IFLS_RX1_8         0x00000000  // RX FIFO >= 1/8 full
#define UART_IFLS_RX2_8         0x00000008  // RX FIFO >= 1/4 full
#define UART_IFLS_RX4_8         0x00000010  // RX FIFO >= 1/2 full (default)
#define UART_IFLS_RX6_8         0x00000018  // RX FIFO >= 3/4 full
#define UART_IFLS_RX7_8         0x00000020  // RX FIFO >= 7/8 full
#define UART_IFLS_TX_M          0x00000007  // UART Transmit Interrupt FIFO
                                            // Level Select
#define UART_IFLS_TX1_8         0x00000000  // TX FIFO <= 1/8 full
#define UART_IFLS_TX2_8         0x00000001  // TX FIFO <= 1/4 full
#define UART_IFLS_TX4_8         0x00000002  // TX FIFO <= 1/2 full (default)
#define UART_IFLS_TX6_8         0x00000003  // TX FIFO <= 3/4 full
#define UART_IFLS_TX7_8         0x00000004  // TX FIFO <= 7/8 full

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IM register.
//
//*****************************************************************************
#define UART_IM_9BITIM          0x00001000  // 9-Bit Mode Interrupt Mask
#define UART_IM_OEIM            0x00000400  // UART Overrun Error Interrupt
                                            // Mask
#define UART_IM_BEIM            0x00000200  // UART Break Error Interrupt Mask
#define UART_IM_PEIM            0x00000100  // UART Parity Error Interrupt Mask
#define UART_IM_FEIM            0x00000080  // UART Framing Error Interrupt
                                            // Mask
#define UART_IM_RTIM            0x00000040  // UART Receive Time-Out Interrupt
                                            // Mask
#define UART_IM_TXIM            0x00000020  // UART Transmit Interrupt Mask
#define UART_IM_RXIM            0x00000010  // UART Receive Interrupt Mask
#define UART_IM_CTSMIM          0x00000002  // UART Clear to Send Modem
                                            // Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
//*****************************************************************************
#define UART_RIS_9BITRIS        0x00001000  // 9-Bit Mode Raw Interrupt Status
#define UART_RIS_OERIS          0x00000400  // UART Overrun Error Raw Interrupt
                                            // Status
#define UART_RIS_BERIS          0x00000200  // UART Break Error Raw Interrupt
                                            // Status
#define UART_RIS_PERIS          0x00000100  // UART Parity Error Raw Interrupt
                                            // Status
#define UART_RIS_FERIS          0x00000080  // UART Framing Error Raw Interrupt
                                            // Status
#define UART_RIS_RTRIS          0x00000040  // UART Receive Time-Out Raw
                                            // Interrupt Status
#define UART_RIS_TXRIS          0x00000020  // UART Transmit Raw Interrupt
                                            // Status
#define UART_RIS_RXRIS          0x00000010  // UART Receive Raw Interrupt
                                            // Status
#define UART_RIS_CTSRIS         0x00000002  // UART Clear to Send Modem Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
//*****************************************************************************
#define UART_MIS_9BITMIS        0x00001000  // 9-Bit Mode Masked Interrupt
                                            // Status
#define UART_MIS_OEMIS          0x00000400  // UART Overrun Error Masked
                                            // Interrupt Status
#define UART_MIS_BEMIS          0x00000200  // UART Break Error Masked
                                            // Interrupt Status
#define UART_MIS_PEMIS          0x00000100  // UART Parity Error Masked
                                            // Interrupt Status
#define UART_MIS_FEMIS          0x00000080  // UART Framing Error Masked
                                            // Interrupt Status
#define UART_MIS_RTMIS          0x00000040  // UART Receive Time-Out Masked
                                            // Interrupt Status
#define UART_MIS_TXMIS          0x00000020  // UART Transmit Masked Interrupt
                                            // Status
#define UART_MIS_RXMIS          0x00000010  // UART Receive Masked Interrupt
                                            // Status
#define UART_MIS_CTSMIS         0x00000002  // UART Clear to Send Modem Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
//*****************************************************************************
#define UART_ICR_9BITIC         0x00001000  // 9-Bit Mode Interrupt Clear
#define UART_ICR_OEIC           0x00000400  // Overrun Error Interrupt Clear
#define UART_ICR_BEIC           0x00000200  // Break Error Interrupt Clear
#define UART_ICR_PEIC           0x00000100  // Parity Error Interrupt Clear
#define UART_ICR_FEIC           0x00000080  // Framing Error Interrupt Clear
#define UART_ICR_RTIC           0x00000040  // Receive Time-Out Interrupt Clear
#define UART_ICR_TXIC           0x00000020  // Transmit Interrupt Clear
#define UART_ICR_RXIC           0x00000010  // Receive Interrupt Clear
#define UART_ICR_CTSMIC         0x00000002  // UART Clear to Send Modem
                                            // Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
//*****************************************************************************
#define UART_DMACTL_DMAERR      0x00000004  // DMA on Error
#define UART_DMACTL_TXDMAE      0x00000002  // Transmit DMA Enable
#define UART_DMACTL_RXDMAE      0x00000001  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITADDR
// register.
//
//*****************************************************************************
#define UART_9BITADDR_9BITEN    0x00008000  // Enable 9-Bit Mode
#define UART_9BITADDR_ADDR_M    0x000000FF  // Self Address for 9-Bit Mode
#define UART_9BITADDR_ADDR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITAMASK
// register.
//
//*****************************************************************************
#define UART_9BITAMASK_MASK_M   0x000000FF  // Self Address Mask for 9-Bit Mode
#define UART_9BITAMASK_MASK_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_PP register.
//
//*****************************************************************************
#define UART_PP_NB              0x00000002  // 9-Bit Support
#define UART_PP_SC              0x00000001  // Smart Card Support

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CC register.
//
//*****************************************************************************
#define UART_CC_CS_M            0x0000000F  // UART Baud Clock Source
#define UART_CC_CS_SYSCLK       0x00000000  // System clock (based on clock
                                            // source and divisor factor)
#define UART_CC_CS_PIOSC        0x00000005  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MSA register.
//
//*****************************************************************************
#define I2C_MSA_SA_M            0x000000FE  // I2C Slave Address
#define I2C_MSA_RS              0x00000001  // Receive not send
#define I2C_MSA_SA_S            1

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCS register.
//
//*****************************************************************************
#define I2C_MCS_CLKTO           0x00000080  // Clock Timeout Error
#define I2C_MCS_BUSBSY          0x00000040  // Bus Busy
#define I2C_MCS_IDLE            0x00000020  // I2C Idle
#define I2C_MCS_ARBLST          0x00000010  // Arbitration Lost
#define I2C_MCS_HS              0x00000010  // High-Speed Enable
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MDR register.
//
//*****************************************************************************
#define I2C_MDR_DATA_M          0x000000FF  // This byte contains the data
                                            // transferred during a transaction
#define I2C_MDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MTPR register.
//
//*****************************************************************************
#define I2C_MTPR_HS             0x00000080  // High-Speed Enable
#define I2C_MTPR_TPR_M          0x0000007F  // Timer Period
#define I2C_MTPR_TPR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
//*****************************************************************************
#define I2C_MIMR_CLKIM          0x00000002  // Clock Timeout Interrupt Mask
#define I2C_MIMR_IM             0x00000001  // Master Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
//*****************************************************************************
#define I2C_MRIS_CLKRIS         0x00000002  // Clock Timeout Raw Interrupt
                                            // Status
#define I2C_MRIS_RIS            0x00000001  // Master Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
//*****************************************************************************
#define I2C_MMIS_CLKMIS         0x00000002  // Clock Timeout Masked Interrupt
                                            // Status
#define I2C_MMIS_MIS            0x00000001  // Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
//*****************************************************************************
#define I2C_MICR_CLKIC          0x00000002  // Clock Timeout Interrupt Clear
#define I2C_MICR_IC             0x00000001  // Master Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
//*****************************************************************************
#define I2C_MCR_GFE             0x00000040  // I2C Glitch Filter Enable
#define I2C_MCR_SFE             0x00000020  // I2C Slave Function Enable
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define I2C_MCR_LPBK            0x00000001  // I2C Loopback

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCLKOCNT register.
//
//*****************************************************************************
#define I2C_MCLKOCNT_CNTL_M     0x000000FF  // I2C Master Count
#define I2C_MCLKOCNT_CNTL_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBMON register.
//
//*****************************************************************************
#define I2C_MBMON_SDA           0x00000002  // I2C SDA Status
#define I2C_MBMON_SCL           0x00000001  // I2C SCL Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR2 register.
//
//*****************************************************************************
#define I2C_MCR2_GFPW_M         0x00000070  // I2C Glitch Filter Pulse Width
#define I2C_MCR2_GFPW_BYPASS    0x00000000  // Bypass
#define I2C_MCR2_GFPW_1         0x00000010  // 1 clock
#define I2C_MCR2_GFPW_2         0x00000020  // 2 clocks
#define I2C_MCR2_GFPW_3         0x00000030  // 3 clocks
#define I2C_MCR2_GFPW_4         0x00000040  // 4 clocks
#define I2C_MCR2_GFPW_8         0x00000050  // 8 clocks
#define I2C_MCR2_GFPW_16        0x00000060  // 16 clocks
#define I2C_MCR2_GFPW_31        0x00000070  // 31 clocks

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR register.
//
//*****************************************************************************
#define I2C_SOAR_OAR_M          0x0000007F  // I2C Slave Own Address
#define I2C_SOAR_OAR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SCSR register.
//
//*****************************************************************************
#define I2C_SCSR_OAR2SEL        0x00000008  // OAR2 Address Matched
#define I2C_SCSR_FBR            0x00000004  // First Byte Received
#define I2C_SCSR_TREQ           0x00000002  // Transmit Request
#define I2C_SCSR_DA             0x00000001  // Device Active
#define I2C_SCSR_RREQ           0x00000001  // Receive Request

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SDR register.
//
//*****************************************************************************
#define I2C_SDR_DATA_M          0x000000FF  // Data for Transfer
#define I2C_SDR_DATA_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SIMR register.
//
//*****************************************************************************
#define I2C_SIMR_STOPIM         0x00000004  // Stop Condition Interrupt Mask
#define I2C_SIMR_STARTIM        0x00000002  // Start Condition Interrupt Mask
#define I2C_SIMR_DATAIM         0x00000001  // Data Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
//*****************************************************************************
#define I2C_SRIS_STOPRIS        0x00000004  // Stop Condition Raw Interrupt
                                            // Status
#define I2C_SRIS_STARTRIS       0x00000002  // Start Condition Raw Interrupt
                                            // Status
#define I2C_SRIS_DATARIS        0x00000001  // Data Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SMIS register.
//
//*****************************************************************************
#define I2C_SMIS_STOPMIS        0x00000004  // Stop Condition Masked Interrupt
                                            // Status
#define I2C_SMIS_STARTMIS       0x00000002  // Start Condition Masked Interrupt
                                            // Status
#define I2C_SMIS_DATAMIS        0x00000001  // Data Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SICR register.
//
//*****************************************************************************
#define I2C_SICR_STOPIC         0x00000004  // Stop Condition Interrupt Clear
#define I2C_SICR_STARTIC        0x00000002  // Start Condition Interrupt Clear
#define I2C_SICR_DATAIC         0x00000001  // Data Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR2 register.
//
//*****************************************************************************
#define I2C_SOAR2_OAR2EN        0x00000080  // I2C Slave Own Address 2 Enable
#define I2C_SOAR2_OAR2_M        0x0000007F  // I2C Slave Own Address 2
#define I2C_SOAR2_OAR2_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SACKCTL register.
//
//*****************************************************************************
#define I2C_SACKCTL_ACKOVAL     0x00000002  // I2C Slave ACK Override Value
#define I2C_SACKCTL_ACKOEN      0x00000001  // I2C Slave ACK Override Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PP register.
//
//*****************************************************************************
#define I2C_PP_HS               0x00000001  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PC register.
//
//*****************************************************************************
#define I2C_PC_HS               0x00000001  // High-Speed Capable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_CTL register.
//
//*****************************************************************************
#define PWM_CTL_GLOBALSYNC3     0x00000008  // Update PWM Generator 3
#define PWM_CTL_GLOBALSYNC2     0x00000004  // Update PWM Generator 2
#define PWM_CTL_GLOBALSYNC1     0x00000002  // Update PWM Generator 1
#define PWM_CTL_GLOBALSYNC0     0x00000001  // Update PWM Generator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_SYNC register.
//
//*****************************************************************************
#define PWM_SYNC_SYNC3          0x00000008  // Reset Generator 3 Counter
#define PWM_SYNC_SYNC2          0x00000004  // Reset Generator 2 Counter
#define PWM_SYNC_SYNC1          0x00000002  // Reset Generator 1 Counter
#define PWM_SYNC_SYNC0          0x00000001  // Reset Generator 0 Counter

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENABLE register.
//
//*****************************************************************************
#define PWM_ENABLE_PWM7EN       0x00000080  // MnPWM7 Output Enable
#define PWM_ENABLE_PWM6EN       0x00000040  // MnPWM6 Output Enable
#define PWM_ENABLE_PWM5EN       0x00000020  // MnPWM5 Output Enable
#define PWM_ENABLE_PWM4EN       0x00000010  // MnPWM4 Output Enable
#define PWM_ENABLE_PWM3EN       0x00000008  // MnPWM3 Output Enable
#define PWM_ENABLE_PWM2EN       0x00000004  // MnPWM2 Output Enable
#define PWM_ENABLE_PWM1EN       0x00000002  // MnPWM1 Output Enable
#define PWM_ENABLE_PWM0EN       0x00000001  // MnPWM0 Output Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INVERT register.
//
//*****************************************************************************
#define PWM_INVERT_PWM7INV      0x00000080  // Invert MnPWM7 Signal
#define PWM_INVERT_PWM6INV      0x00000040  // Invert MnPWM6 Signal
#define PWM_INVERT_PWM5INV      0x00000020  // Invert MnPWM5 Signal
#define PWM_INVERT_PWM4INV      0x00000010  // Invert MnPWM4 Signal
#define PWM_INVERT_PWM3INV      0x00000008  // Invert MnPWM3 Signal
#define PWM_INVERT_PWM2INV      0x00000004  // Invert MnPWM2 Signal
#define PWM_INVERT_PWM1INV      0x00000002  // Invert MnPWM1 Signal
#define PWM_INVERT_PWM0INV      0x00000001  // Invert MnPWM0 Signal

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULT register.
//
//*****************************************************************************
#define PWM_FAULT_FAULT7        0x00000080  // MnPWM7 Fault
#define PWM_FAULT_FAULT6        0x00000040  // MnPWM6 Fault
#define PWM_FAULT_FAULT5        0x00000020  // MnPWM5 Fault
#define PWM_FAULT_FAULT4        0x00000010  // MnPWM4 Fault
#define PWM_FAULT_FAULT3        0x00000008  // MnPWM3 Fault
#define PWM_FAULT_FAULT2        0x00000004  // MnPWM2 Fault
#define PWM_FAULT_FAULT1        0x00000002  // MnPWM1 Fault
#define PWM_FAULT_FAULT0        0x00000001  // MnPWM0 Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_INTEN register.
//
//*****************************************************************************
#define PWM_INTEN_INTFAULT1     0x00020000  // Interrupt Fault 1
#define PWM_INTEN_INTFAULT0     0x00010000  // Interrupt Fault 0
#define PWM_INTEN_INTPWM3       0x00000008  // PWM3 Interrupt Enable
#define PWM_INTEN_INTPWM2       0x00000004  // PWM2 Interrupt Enable
#define PWM_INTEN_INTPWM1       0x00000002  // PWM1 Interrupt Enable
#define PWM_INTEN_INTPWM0       0x00000001  // PWM0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_RIS register.
//
//*****************************************************************************
#define PWM_RIS_INTFAULT1       0x00020000  // Interrupt Fault PWM 1
#define PWM_RIS_INTFAULT0       0x00010000  // Interrupt Fault PWM 0
#define PWM_RIS_INTPWM3         0x00000008  // PWM3 Interrupt Asserted
#define PWM_RIS_INTPWM2         0x00000004  // PWM2 Interrupt Asserted
#define PWM_RIS_INTPWM1         0x00000002  // PWM1 Interrupt Asserted
#define PWM_RIS_INTPWM0         0x00000001  // PWM0 Interrupt Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ISC register.
//
//*****************************************************************************
#define PWM_ISC_INTFAULT1       0x00020000  // FAULT1 Interrupt Asserted
#define PWM_ISC_INTFAULT0       0x00010000  // FAULT0 Interrupt Asserted
#define PWM_ISC_INTPWM3         0x00000008  // PWM3 Interrupt Status
#define PWM_ISC_INTPWM2         0x00000004  // PWM2 Interrupt Status
#define PWM_ISC_INTPWM1         0x00000002  // PWM1 Interrupt Status
#define PWM_ISC_INTPWM0         0x00000001  // PWM0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_STATUS register.
//
//*****************************************************************************
#define PWM_STATUS_FAULT1       0x00000002  // Generator 1 Fault Status
#define PWM_STATUS_FAULT0       0x00000001  // Generator 0 Fault Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_FAULTVAL register.
//
//*****************************************************************************
#define PWM_FAULTVAL_PWM7       0x00000080  // MnPWM7 Fault Value
#define PWM_FAULTVAL_PWM6       0x00000040  // MnPWM6 Fault Value
#define PWM_FAULTVAL_PWM5       0x00000020  // MnPWM5 Fault Value
#define PWM_FAULTVAL_PWM4       0x00000010  // MnPWM4 Fault Value
#define PWM_FAULTVAL_PWM3       0x00000008  // MnPWM3 Fault Value
#define PWM_FAULTVAL_PWM2       0x00000004  // MnPWM2 Fault Value
#define PWM_FAULTVAL_PWM1       0x00000002  // MnPWM1 Fault Value
#define PWM_FAULTVAL_PWM0       0x00000001  // MnPWM0 Fault Value

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_ENUPD register.
//
//*****************************************************************************
#define PWM_ENUPD_ENUPD7_M      0x0000C000  // MnPWM7 Enable Update Mode
#define PWM_ENUPD_ENUPD7_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD7_LSYNC  0x00008000  // Locally Synchronized
#define PWM_ENUPD_ENUPD7_GSYNC  0x0000C000  // Globally Synchronized
#define PWM_ENUPD_ENUPD6_M      0x00003000  // MnPWM6 Enable Update Mode
#define PWM_ENUPD_ENUPD6_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD6_LSYNC  0x00002000  // Locally Synchronized
#define PWM_ENUPD_ENUPD6_GSYNC  0x00003000  // Globally Synchronized
#define PWM_ENUPD_ENUPD5_M      0x00000C00  // MnPWM5 Enable Update Mode
#define PWM_ENUPD_ENUPD5_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD5_LSYNC  0x00000800  // Locally Synchronized
#define PWM_ENUPD_ENUPD5_GSYNC  0x00000C00  // Globally Synchronized
#define PWM_ENUPD_ENUPD4_M      0x00000300  // MnPWM4 Enable Update Mode
#define PWM_ENUPD_ENUPD4_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD4_LSYNC  0x00000200  // Locally Synchronized
#define PWM_ENUPD_ENUPD4_GSYNC  0x00000300  // Globally Synchronized
#define PWM_ENUPD_ENUPD3_M      0x000000C0  // MnPWM3 Enable Update Mode
#define PWM_ENUPD_ENUPD3_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD3_LSYNC  0x00000080  // Locally Synchronized
#define PWM_ENUPD_ENUPD3_GSYNC  0x000000C0  // Globally Synchronized
#define PWM_ENUPD_ENUPD2_M      0x00000030  // MnPWM2 Enable Update Mode
#define PWM_ENUPD_ENUPD2_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD2_LSYNC  0x00000020  // Locally Synchronized
#define PWM_ENUPD_ENUPD2_GSYNC  0x00000030  // Globally Synchronized
#define PWM_ENUPD_ENUPD1_M      0x0000000C  // MnPWM1 Enable Update Mode
#define PWM_ENUPD_ENUPD1_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD1_LSYNC  0x00000008  // Locally Synchronized
#define PWM_ENUPD_ENUPD1_GSYNC  0x0000000C  // Globally Synchronized
#define PWM_ENUPD_ENUPD0_M      0x00000003  // MnPWM0 Enable Update Mode
#define PWM_ENUPD_ENUPD0_IMM    0x00000000  // Immediate
#define PWM_ENUPD_ENUPD0_LSYNC  0x00000002  // Locally Synchronized
#define PWM_ENUPD_ENUPD0_GSYNC  0x00000003  // Globally Synchronized

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CTL register.
//
//*****************************************************************************
#define PWM_0_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_0_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_0_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_0_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_0_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_0_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_0_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_0_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_0_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_0_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_0_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_0_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_0_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_0_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_0_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_0_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_0_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_0_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_0_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_0_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_0_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_0_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_0_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_0_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_0_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_0_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_0_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_0_CTL_MODE          0x00000002  // Counter Mode
#define PWM_0_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_INTEN register.
//
//*****************************************************************************
#define PWM_0_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
                                            // Down
#define PWM_0_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_0_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
                                            // Down
#define PWM_0_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_0_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_0_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_0_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
                                            // Down
#define PWM_0_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
                                            // Up
#define PWM_0_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
                                            // Down
#define PWM_0_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
                                            // Up
#define PWM_0_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_0_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_RIS register.
//
//*****************************************************************************
#define PWM_0_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
                                            // Status
#define PWM_0_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_0_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
                                            // Status
#define PWM_0_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_0_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_0_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_ISC register.
//
//*****************************************************************************
#define PWM_0_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_0_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_0_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_0_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_0_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_0_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_LOAD register.
//
//*****************************************************************************
#define PWM_0_LOAD_M            0x0000FFFF  // Counter Load Value
#define PWM_0_LOAD_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_COUNT register.
//
//*****************************************************************************
#define PWM_0_COUNT_M           0x0000FFFF  // Counter Value
#define PWM_0_COUNT_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPA register.
//
//*****************************************************************************
#define PWM_0_CMPA_M            0x0000FFFF  // Comparator A Value
#define PWM_0_CMPA_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_CMPB register.
//
//*****************************************************************************
#define PWM_0_CMPB_M            0x0000FFFF  // Comparator B Value
#define PWM_0_CMPB_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENA register.
//
//*****************************************************************************
#define PWM_0_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_0_GENA_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_0_GENA_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_0_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_0_GENA_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_0_GENA_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_0_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_0_GENA_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_0_GENA_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_0_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_0_GENA_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_0_GENA_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmA Low
#define PWM_0_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_0_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_0_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_0_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_0_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_0_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_0_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_0_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_0_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_0_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_GENB register.
//
//*****************************************************************************
#define PWM_0_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_0_GENB_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_0_GENB_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_0_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_0_GENB_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_0_GENB_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_0_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_0_GENB_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_0_GENB_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_0_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_0_GENB_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_0_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_0_GENB_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmB Low
#define PWM_0_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_0_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_0_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_0_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_0_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_0_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_0_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_0_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_0_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_0_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBCTL register.
//
//*****************************************************************************
#define PWM_0_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBRISE register.
//
//*****************************************************************************
#define PWM_0_DBRISE_DELAY_M    0x00000FFF  // Dead-Band Rise Delay
#define PWM_0_DBRISE_DELAY_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_DBFALL register.
//
//*****************************************************************************
#define PWM_0_DBFALL_DELAY_M    0x00000FFF  // Dead-Band Fall Delay
#define PWM_0_DBFALL_DELAY_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_0_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_0_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_0_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_0_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_0_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_0_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_0_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_0_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_0_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_0_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_0_MINFLTPER_M       0x0000FFFF  // Minimum Fault Period
#define PWM_0_MINFLTPER_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CTL register.
//
//*****************************************************************************
#define PWM_1_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_1_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_1_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_1_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_1_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_1_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_1_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_1_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_1_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_1_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_1_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_1_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_1_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_1_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_1_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_1_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_1_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_1_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_1_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_1_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_1_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_1_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_1_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_1_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_1_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_1_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_1_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_1_CTL_MODE          0x00000002  // Counter Mode
#define PWM_1_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_INTEN register.
//
//*****************************************************************************
#define PWM_1_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
                                            // Down
#define PWM_1_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_1_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
                                            // Down
#define PWM_1_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_1_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_1_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_1_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
                                            // Down
#define PWM_1_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
                                            // Up
#define PWM_1_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
                                            // Down
#define PWM_1_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
                                            // Up
#define PWM_1_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_1_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_RIS register.
//
//*****************************************************************************
#define PWM_1_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
                                            // Status
#define PWM_1_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_1_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
                                            // Status
#define PWM_1_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_1_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_1_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_ISC register.
//
//*****************************************************************************
#define PWM_1_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_1_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_1_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_1_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_1_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_1_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_LOAD register.
//
//*****************************************************************************
#define PWM_1_LOAD_LOAD_M       0x0000FFFF  // Counter Load Value
#define PWM_1_LOAD_LOAD_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_COUNT register.
//
//*****************************************************************************
#define PWM_1_COUNT_COUNT_M     0x0000FFFF  // Counter Value
#define PWM_1_COUNT_COUNT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPA register.
//
//*****************************************************************************
#define PWM_1_CMPA_COMPA_M      0x0000FFFF  // Comparator A Value
#define PWM_1_CMPA_COMPA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_CMPB register.
//
//*****************************************************************************
#define PWM_1_CMPB_COMPB_M      0x0000FFFF  // Comparator B Value
#define PWM_1_CMPB_COMPB_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENA register.
//
//*****************************************************************************
#define PWM_1_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_1_GENA_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_1_GENA_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_1_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_1_GENA_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_1_GENA_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_1_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_1_GENA_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_1_GENA_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_1_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_1_GENA_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_1_GENA_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmA Low
#define PWM_1_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_1_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_1_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_1_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_1_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_1_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_1_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_1_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_1_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_1_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_1_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_GENB register.
//
//*****************************************************************************
#define PWM_1_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_1_GENB_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_1_GENB_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_1_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_1_GENB_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_1_GENB_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_1_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_1_GENB_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_1_GENB_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_1_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_1_GENB_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_1_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_1_GENB_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmB Low
#define PWM_1_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_1_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_1_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_1_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_1_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_1_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_1_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_1_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_1_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_1_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_1_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBCTL register.
//
//*****************************************************************************
#define PWM_1_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBRISE register.
//
//*****************************************************************************
#define PWM_1_DBRISE_RISEDELAY_M                                              \
                                0x00000FFF  // Dead-Band Rise Delay
#define PWM_1_DBRISE_RISEDELAY_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_DBFALL register.
//
//*****************************************************************************
#define PWM_1_DBFALL_FALLDELAY_M                                              \
                                0x00000FFF  // Dead-Band Fall Delay
#define PWM_1_DBFALL_FALLDELAY_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_1_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_1_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_1_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_1_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_1_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_1_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_1_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_1_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_1_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_1_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_1_MINFLTPER_MFP_M   0x0000FFFF  // Minimum Fault Period
#define PWM_1_MINFLTPER_MFP_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CTL register.
//
//*****************************************************************************
#define PWM_2_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_2_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_2_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_2_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_2_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_2_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_2_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_2_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_2_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_2_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_2_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_2_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_2_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_2_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_2_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_2_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_2_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_2_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_2_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_2_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_2_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_2_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_2_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_2_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_2_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_2_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_2_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_2_CTL_MODE          0x00000002  // Counter Mode
#define PWM_2_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_INTEN register.
//
//*****************************************************************************
#define PWM_2_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
                                            // Down
#define PWM_2_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_2_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
                                            // Down
#define PWM_2_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_2_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_2_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_2_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
                                            // Down
#define PWM_2_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
                                            // Up
#define PWM_2_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
                                            // Down
#define PWM_2_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
                                            // Up
#define PWM_2_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_2_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_RIS register.
//
//*****************************************************************************
#define PWM_2_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
                                            // Status
#define PWM_2_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_2_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
                                            // Status
#define PWM_2_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_2_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_2_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_ISC register.
//
//*****************************************************************************
#define PWM_2_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_2_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_2_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_2_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_2_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_2_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_LOAD register.
//
//*****************************************************************************
#define PWM_2_LOAD_LOAD_M       0x0000FFFF  // Counter Load Value
#define PWM_2_LOAD_LOAD_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_COUNT register.
//
//*****************************************************************************
#define PWM_2_COUNT_COUNT_M     0x0000FFFF  // Counter Value
#define PWM_2_COUNT_COUNT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPA register.
//
//*****************************************************************************
#define PWM_2_CMPA_COMPA_M      0x0000FFFF  // Comparator A Value
#define PWM_2_CMPA_COMPA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_CMPB register.
//
//*****************************************************************************
#define PWM_2_CMPB_COMPB_M      0x0000FFFF  // Comparator B Value
#define PWM_2_CMPB_COMPB_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENA register.
//
//*****************************************************************************
#define PWM_2_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_2_GENA_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_2_GENA_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_2_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_2_GENA_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_2_GENA_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_2_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_2_GENA_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_2_GENA_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_2_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_2_GENA_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_2_GENA_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmA Low
#define PWM_2_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_2_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_2_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_2_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_2_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_2_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_2_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_2_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_2_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_2_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_2_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_GENB register.
//
//*****************************************************************************
#define PWM_2_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_2_GENB_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_2_GENB_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_2_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_2_GENB_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_2_GENB_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_2_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_2_GENB_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_2_GENB_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_2_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_2_GENB_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_2_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_2_GENB_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmB Low
#define PWM_2_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_2_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_2_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_2_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_2_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_2_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_2_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_2_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_2_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_2_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_2_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBCTL register.
//
//*****************************************************************************
#define PWM_2_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBRISE register.
//
//*****************************************************************************
#define PWM_2_DBRISE_RISEDELAY_M                                              \
                                0x00000FFF  // Dead-Band Rise Delay
#define PWM_2_DBRISE_RISEDELAY_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_DBFALL register.
//
//*****************************************************************************
#define PWM_2_DBFALL_FALLDELAY_M                                              \
                                0x00000FFF  // Dead-Band Fall Delay
#define PWM_2_DBFALL_FALLDELAY_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_2_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_2_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_2_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_2_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_2_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_2_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_2_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_2_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_2_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_2_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_2_MINFLTPER_MFP_M   0x0000FFFF  // Minimum Fault Period
#define PWM_2_MINFLTPER_MFP_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CTL register.
//
//*****************************************************************************
#define PWM_3_CTL_LATCH         0x00040000  // Latch Fault Input
#define PWM_3_CTL_MINFLTPER     0x00020000  // Minimum Fault Period
#define PWM_3_CTL_FLTSRC        0x00010000  // Fault Condition Source
#define PWM_3_CTL_DBFALLUPD_M   0x0000C000  // PWMnDBFALL Update Mode
#define PWM_3_CTL_DBFALLUPD_I   0x00000000  // Immediate
#define PWM_3_CTL_DBFALLUPD_LS  0x00008000  // Locally Synchronized
#define PWM_3_CTL_DBFALLUPD_GS  0x0000C000  // Globally Synchronized
#define PWM_3_CTL_DBRISEUPD_M   0x00003000  // PWMnDBRISE Update Mode
#define PWM_3_CTL_DBRISEUPD_I   0x00000000  // Immediate
#define PWM_3_CTL_DBRISEUPD_LS  0x00002000  // Locally Synchronized
#define PWM_3_CTL_DBRISEUPD_GS  0x00003000  // Globally Synchronized
#define PWM_3_CTL_DBCTLUPD_M    0x00000C00  // PWMnDBCTL Update Mode
#define PWM_3_CTL_DBCTLUPD_I    0x00000000  // Immediate
#define PWM_3_CTL_DBCTLUPD_LS   0x00000800  // Locally Synchronized
#define PWM_3_CTL_DBCTLUPD_GS   0x00000C00  // Globally Synchronized
#define PWM_3_CTL_GENBUPD_M     0x00000300  // PWMnGENB Update Mode
#define PWM_3_CTL_GENBUPD_I     0x00000000  // Immediate
#define PWM_3_CTL_GENBUPD_LS    0x00000200  // Locally Synchronized
#define PWM_3_CTL_GENBUPD_GS    0x00000300  // Globally Synchronized
#define PWM_3_CTL_GENAUPD_M     0x000000C0  // PWMnGENA Update Mode
#define PWM_3_CTL_GENAUPD_I     0x00000000  // Immediate
#define PWM_3_CTL_GENAUPD_LS    0x00000080  // Locally Synchronized
#define PWM_3_CTL_GENAUPD_GS    0x000000C0  // Globally Synchronized
#define PWM_3_CTL_CMPBUPD       0x00000020  // Comparator B Update Mode
#define PWM_3_CTL_CMPAUPD       0x00000010  // Comparator A Update Mode
#define PWM_3_CTL_LOADUPD       0x00000008  // Load Register Update Mode
#define PWM_3_CTL_DEBUG         0x00000004  // Debug Mode
#define PWM_3_CTL_MODE          0x00000002  // Counter Mode
#define PWM_3_CTL_ENABLE        0x00000001  // PWM Block Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_INTEN register.
//
//*****************************************************************************
#define PWM_3_INTEN_TRCMPBD     0x00002000  // Trigger for Counter=PWMnCMPB
                                            // Down
#define PWM_3_INTEN_TRCMPBU     0x00001000  // Trigger for Counter=PWMnCMPB Up
#define PWM_3_INTEN_TRCMPAD     0x00000800  // Trigger for Counter=PWMnCMPA
                                            // Down
#define PWM_3_INTEN_TRCMPAU     0x00000400  // Trigger for Counter=PWMnCMPA Up
#define PWM_3_INTEN_TRCNTLOAD   0x00000200  // Trigger for Counter=PWMnLOAD
#define PWM_3_INTEN_TRCNTZERO   0x00000100  // Trigger for Counter=0
#define PWM_3_INTEN_INTCMPBD    0x00000020  // Interrupt for Counter=PWMnCMPB
                                            // Down
#define PWM_3_INTEN_INTCMPBU    0x00000010  // Interrupt for Counter=PWMnCMPB
                                            // Up
#define PWM_3_INTEN_INTCMPAD    0x00000008  // Interrupt for Counter=PWMnCMPA
                                            // Down
#define PWM_3_INTEN_INTCMPAU    0x00000004  // Interrupt for Counter=PWMnCMPA
                                            // Up
#define PWM_3_INTEN_INTCNTLOAD  0x00000002  // Interrupt for Counter=PWMnLOAD
#define PWM_3_INTEN_INTCNTZERO  0x00000001  // Interrupt for Counter=0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_RIS register.
//
//*****************************************************************************
#define PWM_3_RIS_INTCMPBD      0x00000020  // Comparator B Down Interrupt
                                            // Status
#define PWM_3_RIS_INTCMPBU      0x00000010  // Comparator B Up Interrupt Status
#define PWM_3_RIS_INTCMPAD      0x00000008  // Comparator A Down Interrupt
                                            // Status
#define PWM_3_RIS_INTCMPAU      0x00000004  // Comparator A Up Interrupt Status
#define PWM_3_RIS_INTCNTLOAD    0x00000002  // Counter=Load Interrupt Status
#define PWM_3_RIS_INTCNTZERO    0x00000001  // Counter=0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_ISC register.
//
//*****************************************************************************
#define PWM_3_ISC_INTCMPBD      0x00000020  // Comparator B Down Interrupt
#define PWM_3_ISC_INTCMPBU      0x00000010  // Comparator B Up Interrupt
#define PWM_3_ISC_INTCMPAD      0x00000008  // Comparator A Down Interrupt
#define PWM_3_ISC_INTCMPAU      0x00000004  // Comparator A Up Interrupt
#define PWM_3_ISC_INTCNTLOAD    0x00000002  // Counter=Load Interrupt
#define PWM_3_ISC_INTCNTZERO    0x00000001  // Counter=0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_LOAD register.
//
//*****************************************************************************
#define PWM_3_LOAD_LOAD_M       0x0000FFFF  // Counter Load Value
#define PWM_3_LOAD_LOAD_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_COUNT register.
//
//*****************************************************************************
#define PWM_3_COUNT_COUNT_M     0x0000FFFF  // Counter Value
#define PWM_3_COUNT_COUNT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPA register.
//
//*****************************************************************************
#define PWM_3_CMPA_COMPA_M      0x0000FFFF  // Comparator A Value
#define PWM_3_CMPA_COMPA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_CMPB register.
//
//*****************************************************************************
#define PWM_3_CMPB_COMPB_M      0x0000FFFF  // Comparator B Value
#define PWM_3_CMPB_COMPB_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENA register.
//
//*****************************************************************************
#define PWM_3_GENA_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_3_GENA_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPBD_INV 0x00000400  // Invert pwmA
#define PWM_3_GENA_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPBD_ONE 0x00000C00  // Drive pwmA High
#define PWM_3_GENA_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_3_GENA_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPBU_INV 0x00000100  // Invert pwmA
#define PWM_3_GENA_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPBU_ONE 0x00000300  // Drive pwmA High
#define PWM_3_GENA_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_3_GENA_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPAD_INV 0x00000040  // Invert pwmA
#define PWM_3_GENA_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPAD_ONE 0x000000C0  // Drive pwmA High
#define PWM_3_GENA_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_3_GENA_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENA_ACTCMPAU_INV 0x00000010  // Invert pwmA
#define PWM_3_GENA_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmA Low
#define PWM_3_GENA_ACTCMPAU_ONE 0x00000030  // Drive pwmA High
#define PWM_3_GENA_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_3_GENA_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_3_GENA_ACTLOAD_INV  0x00000004  // Invert pwmA
#define PWM_3_GENA_ACTLOAD_ZERO 0x00000008  // Drive pwmA Low
#define PWM_3_GENA_ACTLOAD_ONE  0x0000000C  // Drive pwmA High
#define PWM_3_GENA_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_3_GENA_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_3_GENA_ACTZERO_INV  0x00000001  // Invert pwmA
#define PWM_3_GENA_ACTZERO_ZERO 0x00000002  // Drive pwmA Low
#define PWM_3_GENA_ACTZERO_ONE  0x00000003  // Drive pwmA High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_GENB register.
//
//*****************************************************************************
#define PWM_3_GENB_ACTCMPBD_M   0x00000C00  // Action for Comparator B Down
#define PWM_3_GENB_ACTCMPBD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPBD_INV 0x00000400  // Invert pwmB
#define PWM_3_GENB_ACTCMPBD_ZERO                                              \
                                0x00000800  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPBD_ONE 0x00000C00  // Drive pwmB High
#define PWM_3_GENB_ACTCMPBU_M   0x00000300  // Action for Comparator B Up
#define PWM_3_GENB_ACTCMPBU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPBU_INV 0x00000100  // Invert pwmB
#define PWM_3_GENB_ACTCMPBU_ZERO                                              \
                                0x00000200  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPBU_ONE 0x00000300  // Drive pwmB High
#define PWM_3_GENB_ACTCMPAD_M   0x000000C0  // Action for Comparator A Down
#define PWM_3_GENB_ACTCMPAD_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPAD_INV 0x00000040  // Invert pwmB
#define PWM_3_GENB_ACTCMPAD_ZERO                                              \
                                0x00000080  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPAD_ONE 0x000000C0  // Drive pwmB High
#define PWM_3_GENB_ACTCMPAU_M   0x00000030  // Action for Comparator A Up
#define PWM_3_GENB_ACTCMPAU_NONE                                              \
                                0x00000000  // Do nothing
#define PWM_3_GENB_ACTCMPAU_INV 0x00000010  // Invert pwmB
#define PWM_3_GENB_ACTCMPAU_ZERO                                              \
                                0x00000020  // Drive pwmB Low
#define PWM_3_GENB_ACTCMPAU_ONE 0x00000030  // Drive pwmB High
#define PWM_3_GENB_ACTLOAD_M    0x0000000C  // Action for Counter=LOAD
#define PWM_3_GENB_ACTLOAD_NONE 0x00000000  // Do nothing
#define PWM_3_GENB_ACTLOAD_INV  0x00000004  // Invert pwmB
#define PWM_3_GENB_ACTLOAD_ZERO 0x00000008  // Drive pwmB Low
#define PWM_3_GENB_ACTLOAD_ONE  0x0000000C  // Drive pwmB High
#define PWM_3_GENB_ACTZERO_M    0x00000003  // Action for Counter=0
#define PWM_3_GENB_ACTZERO_NONE 0x00000000  // Do nothing
#define PWM_3_GENB_ACTZERO_INV  0x00000001  // Invert pwmB
#define PWM_3_GENB_ACTZERO_ZERO 0x00000002  // Drive pwmB Low
#define PWM_3_GENB_ACTZERO_ONE  0x00000003  // Drive pwmB High

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBCTL register.
//
//*****************************************************************************
#define PWM_3_DBCTL_ENABLE      0x00000001  // Dead-Band Generator Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBRISE register.
//
//*****************************************************************************
#define PWM_3_DBRISE_RISEDELAY_M                                              \
                                0x00000FFF  // Dead-Band Rise Delay
#define PWM_3_DBRISE_RISEDELAY_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_DBFALL register.
//
//*****************************************************************************
#define PWM_3_DBFALL_FALLDELAY_M                                              \
                                0x00000FFF  // Dead-Band Fall Delay
#define PWM_3_DBFALL_FALLDELAY_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC0
// register.
//
//*****************************************************************************
#define PWM_3_FLTSRC0_FAULT1    0x00000002  // Fault1 Input
#define PWM_3_FLTSRC0_FAULT0    0x00000001  // Fault0 Input

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSRC1
// register.
//
//*****************************************************************************
#define PWM_3_FLTSRC1_DCMP7     0x00000080  // Digital Comparator 7
#define PWM_3_FLTSRC1_DCMP6     0x00000040  // Digital Comparator 6
#define PWM_3_FLTSRC1_DCMP5     0x00000020  // Digital Comparator 5
#define PWM_3_FLTSRC1_DCMP4     0x00000010  // Digital Comparator 4
#define PWM_3_FLTSRC1_DCMP3     0x00000008  // Digital Comparator 3
#define PWM_3_FLTSRC1_DCMP2     0x00000004  // Digital Comparator 2
#define PWM_3_FLTSRC1_DCMP1     0x00000002  // Digital Comparator 1
#define PWM_3_FLTSRC1_DCMP0     0x00000001  // Digital Comparator 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_MINFLTPER
// register.
//
//*****************************************************************************
#define PWM_3_MINFLTPER_MFP_M   0x0000FFFF  // Minimum Fault Period
#define PWM_3_MINFLTPER_MFP_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSEN register.
//
//*****************************************************************************
#define PWM_0_FLTSEN_FAULT1     0x00000002  // Fault1 Sense
#define PWM_0_FLTSEN_FAULT0     0x00000001  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_0_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_0_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_0_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_0_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_0_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_0_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_0_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_0_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_0_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_0_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_0_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSEN register.
//
//*****************************************************************************
#define PWM_1_FLTSEN_FAULT1     0x00000002  // Fault1 Sense
#define PWM_1_FLTSEN_FAULT0     0x00000001  // Fault0 Sense

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_1_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_1_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_1_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_1_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_1_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_1_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_1_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_1_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_1_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_1_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_1_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_2_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_2_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_2_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_2_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_2_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_2_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_2_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_2_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_2_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_2_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_2_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT0
// register.
//
//*****************************************************************************
#define PWM_3_FLTSTAT0_FAULT1   0x00000002  // Fault Input 1
#define PWM_3_FLTSTAT0_FAULT0   0x00000001  // Fault Input 0

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_3_FLTSTAT1
// register.
//
//*****************************************************************************
#define PWM_3_FLTSTAT1_DCMP7    0x00000080  // Digital Comparator 7 Trigger
#define PWM_3_FLTSTAT1_DCMP6    0x00000040  // Digital Comparator 6 Trigger
#define PWM_3_FLTSTAT1_DCMP5    0x00000020  // Digital Comparator 5 Trigger
#define PWM_3_FLTSTAT1_DCMP4    0x00000010  // Digital Comparator 4 Trigger
#define PWM_3_FLTSTAT1_DCMP3    0x00000008  // Digital Comparator 3 Trigger
#define PWM_3_FLTSTAT1_DCMP2    0x00000004  // Digital Comparator 2 Trigger
#define PWM_3_FLTSTAT1_DCMP1    0x00000002  // Digital Comparator 1 Trigger
#define PWM_3_FLTSTAT1_DCMP0    0x00000001  // Digital Comparator 0 Trigger

//*****************************************************************************
//
// The following are defines for the bit fields in the PWM_O_PP register.
//
//*****************************************************************************
#define PWM_PP_ONE              0x00000400  // One-Shot Mode
#define PWM_PP_EFAULT           0x00000200  // Extended Fault
#define PWM_PP_ESYNC            0x00000100  // Extended Synchronization
#define PWM_PP_FCNT_M           0x000000F0  // Fault Inputs (per PWM unit)
#define PWM_PP_GCNT_M           0x0000000F  // Generators
#define PWM_PP_FCNT_S           4
#define PWM_PP_GCNT_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_CTL register.
//
//*****************************************************************************
#define QEI_CTL_FILTCNT_M       0x000F0000  // Input Filter Prescale Count
#define QEI_CTL_FILTEN          0x00002000  // Enable Input Filter
#define QEI_CTL_STALLEN         0x00001000  // Stall QEI
#define QEI_CTL_INVI            0x00000800  // Invert Index Pulse
#define QEI_CTL_INVB            0x00000400  // Invert PhB
#define QEI_CTL_INVA            0x00000200  // Invert PhA
#define QEI_CTL_VELDIV_M        0x000001C0  // Predivide Velocity
#define QEI_CTL_VELDIV_1        0x00000000  // QEI clock /1
#define QEI_CTL_VELDIV_2        0x00000040  // QEI clock /2
#define QEI_CTL_VELDIV_4        0x00000080  // QEI clock /4
#define QEI_CTL_VELDIV_8        0x000000C0  // QEI clock /8
#define QEI_CTL_VELDIV_16       0x00000100  // QEI clock /16
#define QEI_CTL_VELDIV_32       0x00000140  // QEI clock /32
#define QEI_CTL_VELDIV_64       0x00000180  // QEI clock /64
#define QEI_CTL_VELDIV_128      0x000001C0  // QEI clock /128
#define QEI_CTL_VELEN           0x00000020  // Capture Velocity
#define QEI_CTL_RESMODE         0x00000010  // Reset Mode
#define QEI_CTL_CAPMODE         0x00000008  // Capture Mode
#define QEI_CTL_SIGMODE         0x00000004  // Signal Mode
#define QEI_CTL_SWAP            0x00000002  // Swap Signals
#define QEI_CTL_ENABLE          0x00000001  // Enable QEI
#define QEI_CTL_FILTCNT_S       16

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_STAT register.
//
//*****************************************************************************
#define QEI_STAT_DIRECTION      0x00000002  // Direction of Rotation
#define QEI_STAT_ERROR          0x00000001  // Error Detected

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_POS register.
//
//*****************************************************************************
#define QEI_POS_M               0xFFFFFFFF  // Current Position Integrator
                                            // Value
#define QEI_POS_S               0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_MAXPOS register.
//
//*****************************************************************************
#define QEI_MAXPOS_M            0xFFFFFFFF  // Maximum Position Integrator
                                            // Value
#define QEI_MAXPOS_S            0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_LOAD register.
//
//*****************************************************************************
#define QEI_LOAD_M              0xFFFFFFFF  // Velocity Timer Load Value
#define QEI_LOAD_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_TIME register.
//
//*****************************************************************************
#define QEI_TIME_M              0xFFFFFFFF  // Velocity Timer Current Value
#define QEI_TIME_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_COUNT register.
//
//*****************************************************************************
#define QEI_COUNT_M             0xFFFFFFFF  // Velocity Pulse Count
#define QEI_COUNT_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_SPEED register.
//
//*****************************************************************************
#define QEI_SPEED_M             0xFFFFFFFF  // Velocity
#define QEI_SPEED_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_INTEN register.
//
//*****************************************************************************
#define QEI_INTEN_ERROR         0x00000008  // Phase Error Interrupt Enable
#define QEI_INTEN_DIR           0x00000004  // Direction Change Interrupt
                                            // Enable
#define QEI_INTEN_TIMER         0x00000002  // Timer Expires Interrupt Enable
#define QEI_INTEN_INDEX         0x00000001  // Index Pulse Detected Interrupt
                                            // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_RIS register.
//
//*****************************************************************************
#define QEI_RIS_ERROR           0x00000008  // Phase Error Detected
#define QEI_RIS_DIR             0x00000004  // Direction Change Detected
#define QEI_RIS_TIMER           0x00000002  // Velocity Timer Expired
#define QEI_RIS_INDEX           0x00000001  // Index Pulse Asserted

//*****************************************************************************
//
// The following are defines for the bit fields in the QEI_O_ISC register.
//
//*****************************************************************************
#define QEI_ISC_ERROR           0x00000008  // Phase Error Interrupt
#define QEI_ISC_DIR             0x00000004  // Direction Change Interrupt
#define QEI_ISC_TIMER           0x00000002  // Velocity Timer Expired Interrupt
#define QEI_ISC_INDEX           0x00000001  // Index Pulse Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CFG register.
//
//*****************************************************************************
#define TIMER_CFG_M             0x00000007  // GPTM Configuration
#define TIMER_CFG_32_BIT_TIMER  0x00000000  // For a 16/32-bit timer, this
                                            // value selects the 32-bit timer
                                            // configuration
#define TIMER_CFG_32_BIT_RTC    0x00000001  // For a 16/32-bit timer, this
                                            // value selects the 32-bit
                                            // real-time clock (RTC) counter
                                            // configuration
#define TIMER_CFG_16_BIT        0x00000004  // For a 16/32-bit timer, this
                                            // value selects the 16-bit timer
                                            // configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMR register.
//
//*****************************************************************************
#define TIMER_TAMR_TAPLO        0x00000800  // GPTM Timer A PWM Legacy
                                            // Operation
#define TIMER_TAMR_TAMRSU       0x00000400  // GPTM Timer A Match Register
                                            // Update
#define TIMER_TAMR_TAPWMIE      0x00000200  // GPTM Timer A PWM Interrupt
                                            // Enable
#define TIMER_TAMR_TAILD        0x00000100  // GPTM Timer A Interval Load Write
#define TIMER_TAMR_TASNAPS      0x00000080  // GPTM Timer A Snap-Shot Mode
#define TIMER_TAMR_TAWOT        0x00000040  // GPTM Timer A Wait-on-Trigger
#define TIMER_TAMR_TAMIE        0x00000020  // GPTM Timer A Match Interrupt
                                            // Enable
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAAMS        0x00000008  // GPTM Timer A Alternate Mode
                                            // Select
#define TIMER_TAMR_TACMR        0x00000004  // GPTM Timer A Capture Mode
#define TIMER_TAMR_TAMR_M       0x00000003  // GPTM Timer A Mode
#define TIMER_TAMR_TAMR_1_SHOT  0x00000001  // One-Shot Timer mode
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_TAMR_TAMR_CAP     0x00000003  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMR register.
//
//*****************************************************************************
#define TIMER_TBMR_TBPLO        0x00000800  // GPTM Timer B PWM Legacy
                                            // Operation
#define TIMER_TBMR_TBMRSU       0x00000400  // GPTM Timer B Match Register
                                            // Update
#define TIMER_TBMR_TBPWMIE      0x00000200  // GPTM Timer B PWM Interrupt
                                            // Enable
#define TIMER_TBMR_TBILD        0x00000100  // GPTM Timer B Interval Load Write
#define TIMER_TBMR_TBSNAPS      0x00000080  // GPTM Timer B Snap-Shot Mode
#define TIMER_TBMR_TBWOT        0x00000040  // GPTM Timer B Wait-on-Trigger
#define TIMER_TBMR_TBMIE        0x00000020  // GPTM Timer B Match Interrupt
                                            // Enable
#define TIMER_TBMR_TBCDIR       0x00000010  // GPTM Timer B Count Direction
#define TIMER_TBMR_TBAMS        0x00000008  // GPTM Timer B Alternate Mode
                                            // Select
#define TIMER_TBMR_TBCMR        0x00000004  // GPTM Timer B Capture Mode
#define TIMER_TBMR_TBMR_M       0x00000003  // GPTM Timer B Mode
#define TIMER_TBMR_TBMR_1_SHOT  0x00000001  // One-Shot Timer mode
#define TIMER_TBMR_TBMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_TBMR_TBMR_CAP     0x00000003  // Capture mode

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CTL register.
//
//*****************************************************************************
#define TIMER_CTL_TBPWML        0x00004000  // GPTM Timer B PWM Output Level
#define TIMER_CTL_TBOTE         0x00002000  // GPTM Timer B Output Trigger
                                            // Enable
#define TIMER_CTL_TBEVENT_M     0x00000C00  // GPTM Timer B Event Mode
#define TIMER_CTL_TBEVENT_POS   0x00000000  // Positive edge
#define TIMER_CTL_TBEVENT_NEG   0x00000400  // Negative edge
#define TIMER_CTL_TBEVENT_BOTH  0x00000C00  // Both edges
#define TIMER_CTL_TBSTALL       0x00000200  // GPTM Timer B Stall Enable
#define TIMER_CTL_TBEN          0x00000100  // GPTM Timer B Enable
#define TIMER_CTL_TAPWML        0x00000040  // GPTM Timer A PWM Output Level
#define TIMER_CTL_TAOTE         0x00000020  // GPTM Timer A Output Trigger
                                            // Enable
#define TIMER_CTL_RTCEN         0x00000010  // GPTM RTC Stall Enable
#define TIMER_CTL_TAEVENT_M     0x0000000C  // GPTM Timer A Event Mode
#define TIMER_CTL_TAEVENT_POS   0x00000000  // Positive edge
#define TIMER_CTL_TAEVENT_NEG   0x00000004  // Negative edge
#define TIMER_CTL_TAEVENT_BOTH  0x0000000C  // Both edges
#define TIMER_CTL_TASTALL       0x00000002  // GPTM Timer A Stall Enable
#define TIMER_CTL_TAEN          0x00000001  // GPTM Timer A Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_SYNC register.
//
//*****************************************************************************
#define TIMER_SYNC_SYNCWT5_M    0x00C00000  // Synchronize GPTM 32/64-Bit Timer
                                            // 5
#define TIMER_SYNC_SYNCWT5_NONE 0x00000000  // GPTM 32/64-Bit Timer 5 is not
                                            // affected
#define TIMER_SYNC_SYNCWT5_TA   0x00400000  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 5 is
                                            // triggered
#define TIMER_SYNC_SYNCWT5_TB   0x00800000  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 5 is
                                            // triggered
#define TIMER_SYNC_SYNCWT5_TATB 0x00C00000  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 5 is triggered
#define TIMER_SYNC_SYNCWT4_M    0x00300000  // Synchronize GPTM 32/64-Bit Timer
                                            // 4
#define TIMER_SYNC_SYNCWT4_NONE 0x00000000  // GPTM 32/64-Bit Timer 4 is not
                                            // affected
#define TIMER_SYNC_SYNCWT4_TA   0x00100000  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 4 is
                                            // triggered
#define TIMER_SYNC_SYNCWT4_TB   0x00200000  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 4 is
                                            // triggered
#define TIMER_SYNC_SYNCWT4_TATB 0x00300000  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 4 is triggered
#define TIMER_SYNC_SYNCWT3_M    0x000C0000  // Synchronize GPTM 32/64-Bit Timer
                                            // 3
#define TIMER_SYNC_SYNCWT3_NONE 0x00000000  // GPTM 32/64-Bit Timer 3 is not
                                            // affected
#define TIMER_SYNC_SYNCWT3_TA   0x00040000  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 3 is
                                            // triggered
#define TIMER_SYNC_SYNCWT3_TB   0x00080000  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 3 is
                                            // triggered
#define TIMER_SYNC_SYNCWT3_TATB 0x000C0000  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 3 is triggered
#define TIMER_SYNC_SYNCWT2_M    0x00030000  // Synchronize GPTM 32/64-Bit Timer
                                            // 2
#define TIMER_SYNC_SYNCWT2_NONE 0x00000000  // GPTM 32/64-Bit Timer 2 is not
                                            // affected
#define TIMER_SYNC_SYNCWT2_TA   0x00010000  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 2 is
                                            // triggered
#define TIMER_SYNC_SYNCWT2_TB   0x00020000  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 2 is
                                            // triggered
#define TIMER_SYNC_SYNCWT2_TATB 0x00030000  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 2 is triggered
#define TIMER_SYNC_SYNCWT1_M    0x0000C000  // Synchronize GPTM 32/64-Bit Timer
                                            // 1
#define TIMER_SYNC_SYNCWT1_NONE 0x00000000  // GPTM 32/64-Bit Timer 1 is not
                                            // affected
#define TIMER_SYNC_SYNCWT1_TA   0x00004000  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 1 is
                                            // triggered
#define TIMER_SYNC_SYNCWT1_TB   0x00008000  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 1 is
                                            // triggered
#define TIMER_SYNC_SYNCWT1_TATB 0x0000C000  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 1 is triggered
#define TIMER_SYNC_SYNCWT0_M    0x00003000  // Synchronize GPTM 32/64-Bit Timer
                                            // 0
#define TIMER_SYNC_SYNCWT0_NONE 0x00000000  // GPTM 32/64-Bit Timer 0 is not
                                            // affected
#define TIMER_SYNC_SYNCWT0_TA   0x00001000  // A timeout event for Timer A of
                                            // GPTM 32/64-Bit Timer 0 is
                                            // triggered
#define TIMER_SYNC_SYNCWT0_TB   0x00002000  // A timeout event for Timer B of
                                            // GPTM 32/64-Bit Timer 0 is
                                            // triggered
#define TIMER_SYNC_SYNCWT0_TATB 0x00003000  // A timeout event for both Timer A
                                            // and Timer B of GPTM 32/64-Bit
                                            // Timer 0 is triggered
#define TIMER_SYNC_SYNCT5_M     0x00000C00  // Synchronize GPTM Timer 5
#define TIMER_SYNC_SYNCT5_NONE  0x00000000  // GPTM5 is not affected
#define TIMER_SYNC_SYNCT5_TA    0x00000400  // A timeout event for Timer A of
                                            // GPTM5 is triggered
#define TIMER_SYNC_SYNCT5_TB    0x00000800  // A timeout event for Timer B of
                                            // GPTM5 is triggered
#define TIMER_SYNC_SYNCT5_TATB  0x00000C00  // A timeout event for both Timer A
                                            // and Timer B of GPTM5 is
                                            // triggered
#define TIMER_SYNC_SYNCT4_M     0x00000300  // Synchronize GPTM Timer 4
#define TIMER_SYNC_SYNCT4_NONE  0x00000000  // GPTM4 is not affected
#define TIMER_SYNC_SYNCT4_TA    0x00000100  // A timeout event for Timer A of
                                            // GPTM4 is triggered
#define TIMER_SYNC_SYNCT4_TB    0x00000200  // A timeout event for Timer B of
                                            // GPTM4 is triggered
#define TIMER_SYNC_SYNCT4_TATB  0x00000300  // A timeout event for both Timer A
                                            // and Timer B of GPTM4 is
                                            // triggered
#define TIMER_SYNC_SYNCT3_M     0x000000C0  // Synchronize GPTM Timer 3
#define TIMER_SYNC_SYNCT3_NONE  0x00000000  // GPTM3 is not affected
#define TIMER_SYNC_SYNCT3_TA    0x00000040  // A timeout event for Timer A of
                                            // GPTM3 is triggered
#define TIMER_SYNC_SYNCT3_TB    0x00000080  // A timeout event for Timer B of
                                            // GPTM3 is triggered
#define TIMER_SYNC_SYNCT3_TATB  0x000000C0  // A timeout event for both Timer A
                                            // and Timer B of GPTM3 is
                                            // triggered
#define TIMER_SYNC_SYNCT2_M     0x00000030  // Synchronize GPTM Timer 2
#define TIMER_SYNC_SYNCT2_NONE  0x00000000  // GPTM2 is not affected
#define TIMER_SYNC_SYNCT2_TA    0x00000010  // A timeout event for Timer A of
                                            // GPTM2 is triggered
#define TIMER_SYNC_SYNCT2_TB    0x00000020  // A timeout event for Timer B of
                                            // GPTM2 is triggered
#define TIMER_SYNC_SYNCT2_TATB  0x00000030  // A timeout event for both Timer A
                                            // and Timer B of GPTM2 is
                                            // triggered
#define TIMER_SYNC_SYNCT1_M     0x0000000C  // Synchronize GPTM Timer 1
#define TIMER_SYNC_SYNCT1_NONE  0x00000000  // GPTM1 is not affected
#define TIMER_SYNC_SYNCT1_TA    0x00000004  // A timeout event for Timer A of
                                            // GPTM1 is triggered
#define TIMER_SYNC_SYNCT1_TB    0x00000008  // A timeout event for Timer B of
                                            // GPTM1 is triggered
#define TIMER_SYNC_SYNCT1_TATB  0x0000000C  // A timeout event for both Timer A
                                            // and Timer B of GPTM1 is
                                            // triggered
#define TIMER_SYNC_SYNCT0_M     0x00000003  // Synchronize GPTM Timer 0
#define TIMER_SYNC_SYNCT0_NONE  0x00000000  // GPTM0 is not affected
#define TIMER_SYNC_SYNCT0_TA    0x00000001  // A timeout event for Timer A of
                                            // GPTM0 is triggered
#define TIMER_SYNC_SYNCT0_TB    0x00000002  // A timeout event for Timer B of
                                            // GPTM0 is triggered
#define TIMER_SYNC_SYNCT0_TATB  0x00000003  // A timeout event for both Timer A
                                            // and Timer B of GPTM0 is
                                            // triggered

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_IMR register.
//
//*****************************************************************************
#define TIMER_IMR_WUEIM         0x00010000  // 32/64-Bit Wide GPTM Write Update
                                            // Error Interrupt Mask
#define TIMER_IMR_TBMIM         0x00000800  // GPTM Timer B Match Interrupt
                                            // Mask
#define TIMER_IMR_CBEIM         0x00000400  // GPTM Timer B Capture Mode Event
                                            // Interrupt Mask
#define TIMER_IMR_CBMIM         0x00000200  // GPTM Timer B Capture Mode Match
                                            // Interrupt Mask
#define TIMER_IMR_TBTOIM        0x00000100  // GPTM Timer B Time-Out Interrupt
                                            // Mask
#define TIMER_IMR_TAMIM         0x00000010  // GPTM Timer A Match Interrupt
                                            // Mask
#define TIMER_IMR_RTCIM         0x00000008  // GPTM RTC Interrupt Mask
#define TIMER_IMR_CAEIM         0x00000004  // GPTM Timer A Capture Mode Event
                                            // Interrupt Mask
#define TIMER_IMR_CAMIM         0x00000002  // GPTM Timer A Capture Mode Match
                                            // Interrupt Mask
#define TIMER_IMR_TATOIM        0x00000001  // GPTM Timer A Time-Out Interrupt
                                            // Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RIS register.
//
//*****************************************************************************
#define TIMER_RIS_WUERIS        0x00010000  // 32/64-Bit Wide GPTM Write Update
                                            // Error Raw Interrupt Status
#define TIMER_RIS_TBMRIS        0x00000800  // GPTM Timer B Match Raw Interrupt
#define TIMER_RIS_CBERIS        0x00000400  // GPTM Timer B Capture Mode Event
                                            // Raw Interrupt
#define TIMER_RIS_CBMRIS        0x00000200  // GPTM Timer B Capture Mode Match
                                            // Raw Interrupt
#define TIMER_RIS_TBTORIS       0x00000100  // GPTM Timer B Time-Out Raw
                                            // Interrupt
#define TIMER_RIS_TAMRIS        0x00000010  // GPTM Timer A Match Raw Interrupt
#define TIMER_RIS_RTCRIS        0x00000008  // GPTM RTC Raw Interrupt
#define TIMER_RIS_CAERIS        0x00000004  // GPTM Timer A Capture Mode Event
                                            // Raw Interrupt
#define TIMER_RIS_CAMRIS        0x00000002  // GPTM Timer A Capture Mode Match
                                            // Raw Interrupt
#define TIMER_RIS_TATORIS       0x00000001  // GPTM Timer A Time-Out Raw
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_MIS register.
//
//*****************************************************************************
#define TIMER_MIS_WUEMIS        0x00010000  // 32/64-Bit Wide GPTM Write Update
                                            // Error Masked Interrupt Status
#define TIMER_MIS_TBMMIS        0x00000800  // GPTM Timer B Match Masked
                                            // Interrupt
#define TIMER_MIS_CBEMIS        0x00000400  // GPTM Timer B Capture Mode Event
                                            // Masked Interrupt
#define TIMER_MIS_CBMMIS        0x00000200  // GPTM Timer B Capture Mode Match
                                            // Masked Interrupt
#define TIMER_MIS_TBTOMIS       0x00000100  // GPTM Timer B Time-Out Masked
                                            // Interrupt
#define TIMER_MIS_TAMMIS        0x00000010  // GPTM Timer A Match Masked
                                            // Interrupt
#define TIMER_MIS_RTCMIS        0x00000008  // GPTM RTC Masked Interrupt
#define TIMER_MIS_CAEMIS        0x00000004  // GPTM Timer A Capture Mode Event
                                            // Masked Interrupt
#define TIMER_MIS_CAMMIS        0x00000002  // GPTM Timer A Capture Mode Match
                                            // Masked Interrupt
#define TIMER_MIS_TATOMIS       0x00000001  // GPTM Timer A Time-Out Masked
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ICR register.
//
//*****************************************************************************
#define TIMER_ICR_WUECINT       0x00010000  // 32/64-Bit Wide GPTM Write Update
                                            // Error Interrupt Clear
#define TIMER_ICR_TBMCINT       0x00000800  // GPTM Timer B Match Interrupt
                                            // Clear
#define TIMER_ICR_CBECINT       0x00000400  // GPTM Timer B Capture Mode Event
                                            // Interrupt Clear
#define TIMER_ICR_CBMCINT       0x00000200  // GPTM Timer B Capture Mode Match
                                            // Interrupt Clear
#define TIMER_ICR_TBTOCINT      0x00000100  // GPTM Timer B Time-Out Interrupt
                                            // Clear
#define TIMER_ICR_TAMCINT       0x00000010  // GPTM Timer A Match Interrupt
                                            // Clear
#define TIMER_ICR_RTCCINT       0x00000008  // GPTM RTC Interrupt Clear
#define TIMER_ICR_CAECINT       0x00000004  // GPTM Timer A Capture Mode Event
                                            // Interrupt Clear
#define TIMER_ICR_CAMCINT       0x00000002  // GPTM Timer A Capture Mode Match
                                            // Interrupt Clear
#define TIMER_ICR_TATOCINT      0x00000001  // GPTM Timer A Time-Out Raw
                                            // Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAILR register.
//
//*****************************************************************************
#define TIMER_TAILR_M           0xFFFFFFFF  // GPTM Timer A Interval Load
                                            // Register
#define TIMER_TAILR_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBILR register.
//
//*****************************************************************************
#define TIMER_TBILR_M           0xFFFFFFFF  // GPTM Timer B Interval Load
                                            // Register
#define TIMER_TBILR_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMATCHR
// register.
//
//*****************************************************************************
#define TIMER_TAMATCHR_TAMR_M   0xFFFFFFFF  // GPTM Timer A Match Register
#define TIMER_TAMATCHR_TAMR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMATCHR
// register.
//
//*****************************************************************************
#define TIMER_TBMATCHR_TBMR_M   0xFFFFFFFF  // GPTM Timer B Match Register
#define TIMER_TBMATCHR_TBMR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPR register.
//
//*****************************************************************************
#define TIMER_TAPR_TAPSRH_M     0x0000FF00  // GPTM Timer A Prescale High Byte
#define TIMER_TAPR_TAPSR_M      0x000000FF  // GPTM Timer A Prescale
#define TIMER_TAPR_TAPSRH_S     8
#define TIMER_TAPR_TAPSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPR register.
//
//*****************************************************************************
#define TIMER_TBPR_TBPSRH_M     0x0000FF00  // GPTM Timer B Prescale High Byte
#define TIMER_TBPR_TBPSR_M      0x000000FF  // GPTM Timer B Prescale
#define TIMER_TBPR_TBPSRH_S     8
#define TIMER_TBPR_TBPSR_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPMR register.
//
//*****************************************************************************
#define TIMER_TAPMR_TAPSMRH_M   0x0000FF00  // GPTM Timer A Prescale Match High
                                            // Byte
#define TIMER_TAPMR_TAPSMR_M    0x000000FF  // GPTM TimerA Prescale Match
#define TIMER_TAPMR_TAPSMRH_S   8
#define TIMER_TAPMR_TAPSMR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPMR register.
//
//*****************************************************************************
#define TIMER_TBPMR_TBPSMRH_M   0x0000FF00  // GPTM Timer B Prescale Match High
                                            // Byte
#define TIMER_TBPMR_TBPSMR_M    0x000000FF  // GPTM TimerB Prescale Match
#define TIMER_TBPMR_TBPSMRH_S   8
#define TIMER_TBPMR_TBPSMR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAR register.
//
//*****************************************************************************
#define TIMER_TAR_M             0xFFFFFFFF  // GPTM Timer A Register
#define TIMER_TAR_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBR register.
//
//*****************************************************************************
#define TIMER_TBR_M             0xFFFFFFFF  // GPTM Timer B Register
#define TIMER_TBR_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAV register.
//
//*****************************************************************************
#define TIMER_TAV_M             0xFFFFFFFF  // GPTM Timer A Value
#define TIMER_TAV_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBV register.
//
//*****************************************************************************
#define TIMER_TBV_M             0xFFFFFFFF  // GPTM Timer B Value
#define TIMER_TBV_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RTCPD register.
//
//*****************************************************************************
#define TIMER_RTCPD_RTCPD_M     0x0000FFFF  // RTC Predivide Counter Value
#define TIMER_RTCPD_RTCPD_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPS register.
//
//*****************************************************************************
#define TIMER_TAPS_PSS_M        0x0000FFFF  // GPTM Timer A Prescaler Snapshot
#define TIMER_TAPS_PSS_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPS register.
//
//*****************************************************************************
#define TIMER_TBPS_PSS_M        0x0000FFFF  // GPTM Timer A Prescaler Value
#define TIMER_TBPS_PSS_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPV register.
//
//*****************************************************************************
#define TIMER_TAPV_PSV_M        0x0000FFFF  // GPTM Timer A Prescaler Value
#define TIMER_TAPV_PSV_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPV register.
//
//*****************************************************************************
#define TIMER_TBPV_PSV_M        0x0000FFFF  // GPTM Timer B Prescaler Value
#define TIMER_TBPV_PSV_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_PP register.
//
//*****************************************************************************
#define TIMER_PP_SIZE_M         0x0000000F  // Count Size
#define TIMER_PP_SIZE_16        0x00000000  // Timer A and Timer B counters are
                                            // 16 bits each with an 8-bit
                                            // prescale counter
#define TIMER_PP_SIZE_32        0x00000001  // Timer A and Timer B counters are
                                            // 32 bits each with a 16-bit
                                            // prescale counter

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ACTSS register.
//
//*****************************************************************************
#define ADC_ACTSS_BUSY          0x00010000  // ADC Busy
#define ADC_ACTSS_ASEN3         0x00000008  // ADC SS3 Enable
#define ADC_ACTSS_ASEN2         0x00000004  // ADC SS2 Enable
#define ADC_ACTSS_ASEN1         0x00000002  // ADC SS1 Enable
#define ADC_ACTSS_ASEN0         0x00000001  // ADC SS0 Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_RIS register.
//
//*****************************************************************************
#define ADC_RIS_INRDC           0x00010000  // Digital Comparator Raw Interrupt
                                            // Status
#define ADC_RIS_INR3            0x00000008  // SS3 Raw Interrupt Status
#define ADC_RIS_INR2            0x00000004  // SS2 Raw Interrupt Status
#define ADC_RIS_INR1            0x00000002  // SS1 Raw Interrupt Status
#define ADC_RIS_INR0            0x00000001  // SS0 Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_IM register.
//
//*****************************************************************************
#define ADC_IM_DCONSS3          0x00080000  // Digital Comparator Interrupt on
                                            // SS3
#define ADC_IM_DCONSS2          0x00040000  // Digital Comparator Interrupt on
                                            // SS2
#define ADC_IM_DCONSS1          0x00020000  // Digital Comparator Interrupt on
                                            // SS1
#define ADC_IM_DCONSS0          0x00010000  // Digital Comparator Interrupt on
                                            // SS0
#define ADC_IM_MASK3            0x00000008  // SS3 Interrupt Mask
#define ADC_IM_MASK2            0x00000004  // SS2 Interrupt Mask
#define ADC_IM_MASK1            0x00000002  // SS1 Interrupt Mask
#define ADC_IM_MASK0            0x00000001  // SS0 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ISC register.
//
//*****************************************************************************
#define ADC_ISC_DCINSS3         0x00080000  // Digital Comparator Interrupt
                                            // Status on SS3
#define ADC_ISC_DCINSS2         0x00040000  // Digital Comparator Interrupt
                                            // Status on SS2
#define ADC_ISC_DCINSS1         0x00020000  // Digital Comparator Interrupt
                                            // Status on SS1
#define ADC_ISC_DCINSS0         0x00010000  // Digital Comparator Interrupt
                                            // Status on SS0
#define ADC_ISC_IN3             0x00000008  // SS3 Interrupt Status and Clear
#define ADC_ISC_IN2             0x00000004  // SS2 Interrupt Status and Clear
#define ADC_ISC_IN1             0x00000002  // SS1 Interrupt Status and Clear
#define ADC_ISC_IN0             0x00000001  // SS0 Interrupt Status and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_OSTAT register.
//
//*****************************************************************************
#define ADC_OSTAT_OV3           0x00000008  // SS3 FIFO Overflow
#define ADC_OSTAT_OV2           0x00000004  // SS2 FIFO Overflow
#define ADC_OSTAT_OV1           0x00000002  // SS1 FIFO Overflow
#define ADC_OSTAT_OV0           0x00000001  // SS0 FIFO Overflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_EMUX register.
//
//*****************************************************************************
#define ADC_EMUX_EM3_M          0x0000F000  // SS3 Trigger Select
#define ADC_EMUX_EM3_PROCESSOR  0x00000000  // Processor (default)
#define ADC_EMUX_EM3_COMP0      0x00001000  // Analog Comparator 0
#define ADC_EMUX_EM3_COMP1      0x00002000  // Analog Comparator 1
#define ADC_EMUX_EM3_EXTERNAL   0x00004000  // External (GPIO Pins)
#define ADC_EMUX_EM3_TIMER      0x00005000  // Timer
#define ADC_EMUX_EM3_PWM0       0x00006000  // PWM generator 0
#define ADC_EMUX_EM3_PWM1       0x00007000  // PWM generator 1
#define ADC_EMUX_EM3_PWM2       0x00008000  // PWM generator 2
#define ADC_EMUX_EM3_PWM3       0x00009000  // PWM generator 3
#define ADC_EMUX_EM3_ALWAYS     0x0000F000  // Always (continuously sample)
#define ADC_EMUX_EM2_M          0x00000F00  // SS2 Trigger Select
#define ADC_EMUX_EM2_PROCESSOR  0x00000000  // Processor (default)
#define ADC_EMUX_EM2_COMP0      0x00000100  // Analog Comparator 0
#define ADC_EMUX_EM2_COMP1      0x00000200  // Analog Comparator 1
#define ADC_EMUX_EM2_EXTERNAL   0x00000400  // External (GPIO Pins)
#define ADC_EMUX_EM2_TIMER      0x00000500  // Timer
#define ADC_EMUX_EM2_PWM0       0x00000600  // PWM generator 0
#define ADC_EMUX_EM2_PWM1       0x00000700  // PWM generator 1
#define ADC_EMUX_EM2_PWM2       0x00000800  // PWM generator 2
#define ADC_EMUX_EM2_PWM3       0x00000900  // PWM generator 3
#define ADC_EMUX_EM2_ALWAYS     0x00000F00  // Always (continuously sample)
#define ADC_EMUX_EM1_M          0x000000F0  // SS1 Trigger Select
#define ADC_EMUX_EM1_PROCESSOR  0x00000000  // Processor (default)
#define ADC_EMUX_EM1_COMP0      0x00000010  // Analog Comparator 0
#define ADC_EMUX_EM1_COMP1      0x00000020  // Analog Comparator 1
#define ADC_EMUX_EM1_EXTERNAL   0x00000040  // External (GPIO Pins)
#define ADC_EMUX_EM1_TIMER      0x00000050  // Timer
#define ADC_EMUX_EM1_PWM0       0x00000060  // PWM generator 0
#define ADC_EMUX_EM1_PWM1       0x00000070  // PWM generator 1
#define ADC_EMUX_EM1_PWM2       0x00000080  // PWM generator 2
#define ADC_EMUX_EM1_PWM3       0x00000090  // PWM generator 3
#define ADC_EMUX_EM1_ALWAYS     0x000000F0  // Always (continuously sample)
#define ADC_EMUX_EM0_M          0x0000000F  // SS0 Trigger Select
#define ADC_EMUX_EM0_PROCESSOR  0x00000000  // Processor (default)
#define ADC_EMUX_EM0_COMP0      0x00000001  // Analog Comparator 0
#define ADC_EMUX_EM0_COMP1      0x00000002  // Analog Comparator 1
#define ADC_EMUX_EM0_EXTERNAL   0x00000004  // External (GPIO Pins)
#define ADC_EMUX_EM0_TIMER      0x00000005  // Timer
#define ADC_EMUX_EM0_PWM0       0x00000006  // PWM generator 0
#define ADC_EMUX_EM0_PWM1       0x00000007  // PWM generator 1
#define ADC_EMUX_EM0_PWM2       0x00000008  // PWM generator 2
#define ADC_EMUX_EM0_PWM3       0x00000009  // PWM generator 3
#define ADC_EMUX_EM0_ALWAYS     0x0000000F  // Always (continuously sample)

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_USTAT register.
//
//*****************************************************************************
#define ADC_USTAT_UV3           0x00000008  // SS3 FIFO Underflow
#define ADC_USTAT_UV2           0x00000004  // SS2 FIFO Underflow
#define ADC_USTAT_UV1           0x00000002  // SS1 FIFO Underflow
#define ADC_USTAT_UV0           0x00000001  // SS0 FIFO Underflow

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_TSSEL register.
//
//*****************************************************************************
#define ADC_TSSEL_PS3_M         0x30000000  // Generator 3 PWM Module Trigger
                                            // Select
#define ADC_TSSEL_PS3_0         0x00000000  // Use Generator 3 (and its
                                            // trigger) in PWM module 0
#define ADC_TSSEL_PS3_1         0x10000000  // Use Generator 3 (and its
                                            // trigger) in PWM module 1
#define ADC_TSSEL_PS2_M         0x00300000  // Generator 2 PWM Module Trigger
                                            // Select
#define ADC_TSSEL_PS2_0         0x00000000  // Use Generator 2 (and its
                                            // trigger) in PWM module 0
#define ADC_TSSEL_PS2_1         0x00100000  // Use Generator 2 (and its
                                            // trigger) in PWM module 1
#define ADC_TSSEL_PS1_M         0x00003000  // Generator 1 PWM Module Trigger
                                            // Select
#define ADC_TSSEL_PS1_0         0x00000000  // Use Generator 1 (and its
                                            // trigger) in PWM module 0
#define ADC_TSSEL_PS1_1         0x00001000  // Use Generator 1 (and its
                                            // trigger) in PWM module 1
#define ADC_TSSEL_PS0_M         0x00000030  // Generator 0 PWM Module Trigger
                                            // Select
#define ADC_TSSEL_PS0_0         0x00000000  // Use Generator 0 (and its
                                            // trigger) in PWM module 0
#define ADC_TSSEL_PS0_1         0x00000010  // Use Generator 0 (and its
                                            // trigger) in PWM module 1

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSPRI register.
//
//*****************************************************************************
#define ADC_SSPRI_SS3_M         0x00003000  // SS3 Priority
#define ADC_SSPRI_SS2_M         0x00000300  // SS2 Priority
#define ADC_SSPRI_SS1_M         0x00000030  // SS1 Priority
#define ADC_SSPRI_SS0_M         0x00000003  // SS0 Priority

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SPC register.
//
//*****************************************************************************
#define ADC_SPC_PHASE_M         0x0000000F  // Phase Difference
#define ADC_SPC_PHASE_0         0x00000000  // ADC sample lags by 0.0
#define ADC_SPC_PHASE_22_5      0x00000001  // ADC sample lags by 22.5
#define ADC_SPC_PHASE_45        0x00000002  // ADC sample lags by 45.0
#define ADC_SPC_PHASE_67_5      0x00000003  // ADC sample lags by 67.5
#define ADC_SPC_PHASE_90        0x00000004  // ADC sample lags by 90.0
#define ADC_SPC_PHASE_112_5     0x00000005  // ADC sample lags by 112.5
#define ADC_SPC_PHASE_135       0x00000006  // ADC sample lags by 135.0
#define ADC_SPC_PHASE_157_5     0x00000007  // ADC sample lags by 157.5
#define ADC_SPC_PHASE_180       0x00000008  // ADC sample lags by 180.0
#define ADC_SPC_PHASE_202_5     0x00000009  // ADC sample lags by 202.5
#define ADC_SPC_PHASE_225       0x0000000A  // ADC sample lags by 225.0
#define ADC_SPC_PHASE_247_5     0x0000000B  // ADC sample lags by 247.5
#define ADC_SPC_PHASE_270       0x0000000C  // ADC sample lags by 270.0
#define ADC_SPC_PHASE_292_5     0x0000000D  // ADC sample lags by 292.5
#define ADC_SPC_PHASE_315       0x0000000E  // ADC sample lags by 315.0
#define ADC_SPC_PHASE_337_5     0x0000000F  // ADC sample lags by 337.5

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PSSI register.
//
//*****************************************************************************
#define ADC_PSSI_GSYNC          0x80000000  // Global Synchronize
#define ADC_PSSI_SYNCWAIT       0x08000000  // Synchronize Wait
#define ADC_PSSI_SS3            0x00000008  // SS3 Initiate
#define ADC_PSSI_SS2            0x00000004  // SS2 Initiate
#define ADC_PSSI_SS1            0x00000002  // SS1 Initiate
#define ADC_PSSI_SS0            0x00000001  // SS0 Initiate

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SAC register.
//
//*****************************************************************************
#define ADC_SAC_AVG_M           0x00000007  // Hardware Averaging Control
#define ADC_SAC_AVG_OFF         0x00000000  // No hardware oversampling
#define ADC_SAC_AVG_2X          0x00000001  // 2x hardware oversampling
#define ADC_SAC_AVG_4X          0x00000002  // 4x hardware oversampling
#define ADC_SAC_AVG_8X          0x00000003  // 8x hardware oversampling
#define ADC_SAC_AVG_16X         0x00000004  // 16x hardware oversampling
#define ADC_SAC_AVG_32X         0x00000005  // 32x hardware oversampling
#define ADC_SAC_AVG_64X         0x00000006  // 64x hardware oversampling

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCISC register.
//
//*****************************************************************************
#define ADC_DCISC_DCINT7        0x00000080  // Digital Comparator 7 Interrupt
                                            // Status and Clear
#define ADC_DCISC_DCINT6        0x00000040  // Digital Comparator 6 Interrupt
                                            // Status and Clear
#define ADC_DCISC_DCINT5        0x00000020  // Digital Comparator 5 Interrupt
                                            // Status and Clear
#define ADC_DCISC_DCINT4        0x00000010  // Digital Comparator 4 Interrupt
                                            // Status and Clear
#define ADC_DCISC_DCINT3        0x00000008  // Digital Comparator 3 Interrupt
                                            // Status and Clear
#define ADC_DCISC_DCINT2        0x00000004  // Digital Comparator 2 Interrupt
                                            // Status and Clear
#define ADC_DCISC_DCINT1        0x00000002  // Digital Comparator 1 Interrupt
                                            // Status and Clear
#define ADC_DCISC_DCINT0        0x00000001  // Digital Comparator 0 Interrupt
                                            // Status and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CTL register.
//
//*****************************************************************************
#define ADC_CTL_VREF_M          0x00000001  // Voltage Reference Select
#define ADC_CTL_VREF_INTERNAL   0x00000000  // VDDA and GNDA are the voltage
                                            // references

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX0 register.
//
//*****************************************************************************
#define ADC_SSMUX0_MUX7_M       0xF0000000  // 8th Sample Input Select
#define ADC_SSMUX0_MUX6_M       0x0F000000  // 7th Sample Input Select
#define ADC_SSMUX0_MUX5_M       0x00F00000  // 6th Sample Input Select
#define ADC_SSMUX0_MUX4_M       0x000F0000  // 5th Sample Input Select
#define ADC_SSMUX0_MUX3_M       0x0000F000  // 4th Sample Input Select
#define ADC_SSMUX0_MUX2_M       0x00000F00  // 3rd Sample Input Select
#define ADC_SSMUX0_MUX1_M       0x000000F0  // 2nd Sample Input Select
#define ADC_SSMUX0_MUX0_M       0x0000000F  // 1st Sample Input Select
#define ADC_SSMUX0_MUX7_S       28
#define ADC_SSMUX0_MUX6_S       24
#define ADC_SSMUX0_MUX5_S       20
#define ADC_SSMUX0_MUX4_S       16
#define ADC_SSMUX0_MUX3_S       12
#define ADC_SSMUX0_MUX2_S       8
#define ADC_SSMUX0_MUX1_S       4
#define ADC_SSMUX0_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL0 register.
//
//*****************************************************************************
#define ADC_SSCTL0_TS7          0x80000000  // 8th Sample Temp Sensor Select
#define ADC_SSCTL0_IE7          0x40000000  // 8th Sample Interrupt Enable
#define ADC_SSCTL0_END7         0x20000000  // 8th Sample is End of Sequence
#define ADC_SSCTL0_D7           0x10000000  // 8th Sample Differential Input
                                            // Select
#define ADC_SSCTL0_TS6          0x08000000  // 7th Sample Temp Sensor Select
#define ADC_SSCTL0_IE6          0x04000000  // 7th Sample Interrupt Enable
#define ADC_SSCTL0_END6         0x02000000  // 7th Sample is End of Sequence
#define ADC_SSCTL0_D6           0x01000000  // 7th Sample Differential Input
                                            // Select
#define ADC_SSCTL0_TS5          0x00800000  // 6th Sample Temp Sensor Select
#define ADC_SSCTL0_IE5          0x00400000  // 6th Sample Interrupt Enable
#define ADC_SSCTL0_END5         0x00200000  // 6th Sample is End of Sequence
#define ADC_SSCTL0_D5           0x00100000  // 6th Sample Differential Input
                                            // Select
#define ADC_SSCTL0_TS4          0x00080000  // 5th Sample Temp Sensor Select
#define ADC_SSCTL0_IE4          0x00040000  // 5th Sample Interrupt Enable
#define ADC_SSCTL0_END4         0x00020000  // 5th Sample is End of Sequence
#define ADC_SSCTL0_D4           0x00010000  // 5th Sample Differential Input
                                            // Select
#define ADC_SSCTL0_TS3          0x00008000  // 4th Sample Temp Sensor Select
#define ADC_SSCTL0_IE3          0x00004000  // 4th Sample Interrupt Enable
#define ADC_SSCTL0_END3         0x00002000  // 4th Sample is End of Sequence
#define ADC_SSCTL0_D3           0x00001000  // 4th Sample Differential Input
                                            // Select
#define ADC_SSCTL0_TS2          0x00000800  // 3rd Sample Temp Sensor Select
#define ADC_SSCTL0_IE2          0x00000400  // 3rd Sample Interrupt Enable
#define ADC_SSCTL0_END2         0x00000200  // 3rd Sample is End of Sequence
#define ADC_SSCTL0_D2           0x00000100  // 3rd Sample Differential Input
                                            // Select
#define ADC_SSCTL0_TS1          0x00000080  // 2nd Sample Temp Sensor Select
#define ADC_SSCTL0_IE1          0x00000040  // 2nd Sample Interrupt Enable
#define ADC_SSCTL0_END1         0x00000020  // 2nd Sample is End of Sequence
#define ADC_SSCTL0_D1           0x00000010  // 2nd Sample Differential Input
                                            // Select
#define ADC_SSCTL0_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL0_IE0          0x00000004  // 1st Sample Interrupt Enable
#define ADC_SSCTL0_END0         0x00000002  // 1st Sample is End of Sequence
#define ADC_SSCTL0_D0           0x00000001  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO0 register.
//
//*****************************************************************************
#define ADC_SSFIFO0_DATA_M      0x00000FFF  // Conversion Result Data
#define ADC_SSFIFO0_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT0 register.
//
//*****************************************************************************
#define ADC_SSFSTAT0_FULL       0x00001000  // FIFO Full
#define ADC_SSFSTAT0_EMPTY      0x00000100  // FIFO Empty
#define ADC_SSFSTAT0_HPTR_M     0x000000F0  // FIFO Head Pointer
#define ADC_SSFSTAT0_TPTR_M     0x0000000F  // FIFO Tail Pointer
#define ADC_SSFSTAT0_HPTR_S     4
#define ADC_SSFSTAT0_TPTR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP0 register.
//
//*****************************************************************************
#define ADC_SSOP0_S7DCOP        0x10000000  // Sample 7 Digital Comparator
                                            // Operation
#define ADC_SSOP0_S6DCOP        0x01000000  // Sample 6 Digital Comparator
                                            // Operation
#define ADC_SSOP0_S5DCOP        0x00100000  // Sample 5 Digital Comparator
                                            // Operation
#define ADC_SSOP0_S4DCOP        0x00010000  // Sample 4 Digital Comparator
                                            // Operation
#define ADC_SSOP0_S3DCOP        0x00001000  // Sample 3 Digital Comparator
                                            // Operation
#define ADC_SSOP0_S2DCOP        0x00000100  // Sample 2 Digital Comparator
                                            // Operation
#define ADC_SSOP0_S1DCOP        0x00000010  // Sample 1 Digital Comparator
                                            // Operation
#define ADC_SSOP0_S0DCOP        0x00000001  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC0 register.
//
//*****************************************************************************
#define ADC_SSDC0_S7DCSEL_M     0xF0000000  // Sample 7 Digital Comparator
                                            // Select
#define ADC_SSDC0_S6DCSEL_M     0x0F000000  // Sample 6 Digital Comparator
                                            // Select
#define ADC_SSDC0_S5DCSEL_M     0x00F00000  // Sample 5 Digital Comparator
                                            // Select
#define ADC_SSDC0_S4DCSEL_M     0x000F0000  // Sample 4 Digital Comparator
                                            // Select
#define ADC_SSDC0_S3DCSEL_M     0x0000F000  // Sample 3 Digital Comparator
                                            // Select
#define ADC_SSDC0_S2DCSEL_M     0x00000F00  // Sample 2 Digital Comparator
                                            // Select
#define ADC_SSDC0_S1DCSEL_M     0x000000F0  // Sample 1 Digital Comparator
                                            // Select
#define ADC_SSDC0_S0DCSEL_M     0x0000000F  // Sample 0 Digital Comparator
                                            // Select
#define ADC_SSDC0_S6DCSEL_S     24
#define ADC_SSDC0_S5DCSEL_S     20
#define ADC_SSDC0_S4DCSEL_S     16
#define ADC_SSDC0_S3DCSEL_S     12
#define ADC_SSDC0_S2DCSEL_S     8
#define ADC_SSDC0_S1DCSEL_S     4
#define ADC_SSDC0_S0DCSEL_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX1 register.
//
//*****************************************************************************
#define ADC_SSMUX1_MUX3_M       0x0000F000  // 4th Sample Input Select
#define ADC_SSMUX1_MUX2_M       0x00000F00  // 3rd Sample Input Select
#define ADC_SSMUX1_MUX1_M       0x000000F0  // 2nd Sample Input Select
#define ADC_SSMUX1_MUX0_M       0x0000000F  // 1st Sample Input Select
#define ADC_SSMUX1_MUX3_S       12
#define ADC_SSMUX1_MUX2_S       8
#define ADC_SSMUX1_MUX1_S       4
#define ADC_SSMUX1_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL1 register.
//
//*****************************************************************************
#define ADC_SSCTL1_TS3          0x00008000  // 4th Sample Temp Sensor Select
#define ADC_SSCTL1_IE3          0x00004000  // 4th Sample Interrupt Enable
#define ADC_SSCTL1_END3         0x00002000  // 4th Sample is End of Sequence
#define ADC_SSCTL1_D3           0x00001000  // 4th Sample Differential Input
                                            // Select
#define ADC_SSCTL1_TS2          0x00000800  // 3rd Sample Temp Sensor Select
#define ADC_SSCTL1_IE2          0x00000400  // 3rd Sample Interrupt Enable
#define ADC_SSCTL1_END2         0x00000200  // 3rd Sample is End of Sequence
#define ADC_SSCTL1_D2           0x00000100  // 3rd Sample Differential Input
                                            // Select
#define ADC_SSCTL1_TS1          0x00000080  // 2nd Sample Temp Sensor Select
#define ADC_SSCTL1_IE1          0x00000040  // 2nd Sample Interrupt Enable
#define ADC_SSCTL1_END1         0x00000020  // 2nd Sample is End of Sequence
#define ADC_SSCTL1_D1           0x00000010  // 2nd Sample Differential Input
                                            // Select
#define ADC_SSCTL1_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL1_IE0          0x00000004  // 1st Sample Interrupt Enable
#define ADC_SSCTL1_END0         0x00000002  // 1st Sample is End of Sequence
#define ADC_SSCTL1_D0           0x00000001  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO1 register.
//
//*****************************************************************************
#define ADC_SSFIFO1_DATA_M      0x00000FFF  // Conversion Result Data
#define ADC_SSFIFO1_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT1 register.
//
//*****************************************************************************
#define ADC_SSFSTAT1_FULL       0x00001000  // FIFO Full
#define ADC_SSFSTAT1_EMPTY      0x00000100  // FIFO Empty
#define ADC_SSFSTAT1_HPTR_M     0x000000F0  // FIFO Head Pointer
#define ADC_SSFSTAT1_TPTR_M     0x0000000F  // FIFO Tail Pointer
#define ADC_SSFSTAT1_HPTR_S     4
#define ADC_SSFSTAT1_TPTR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP1 register.
//
//*****************************************************************************
#define ADC_SSOP1_S3DCOP        0x00001000  // Sample 3 Digital Comparator
                                            // Operation
#define ADC_SSOP1_S2DCOP        0x00000100  // Sample 2 Digital Comparator
                                            // Operation
#define ADC_SSOP1_S1DCOP        0x00000010  // Sample 1 Digital Comparator
                                            // Operation
#define ADC_SSOP1_S0DCOP        0x00000001  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC1 register.
//
//*****************************************************************************
#define ADC_SSDC1_S3DCSEL_M     0x0000F000  // Sample 3 Digital Comparator
                                            // Select
#define ADC_SSDC1_S2DCSEL_M     0x00000F00  // Sample 2 Digital Comparator
                                            // Select
#define ADC_SSDC1_S1DCSEL_M     0x000000F0  // Sample 1 Digital Comparator
                                            // Select
#define ADC_SSDC1_S0DCSEL_M     0x0000000F  // Sample 0 Digital Comparator
                                            // Select
#define ADC_SSDC1_S2DCSEL_S     8
#define ADC_SSDC1_S1DCSEL_S     4
#define ADC_SSDC1_S0DCSEL_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX2 register.
//
//*****************************************************************************
#define ADC_SSMUX2_MUX3_M       0x0000F000  // 4th Sample Input Select
#define ADC_SSMUX2_MUX2_M       0x00000F00  // 3rd Sample Input Select
#define ADC_SSMUX2_MUX1_M       0x000000F0  // 2nd Sample Input Select
#define ADC_SSMUX2_MUX0_M       0x0000000F  // 1st Sample Input Select
#define ADC_SSMUX2_MUX3_S       12
#define ADC_SSMUX2_MUX2_S       8
#define ADC_SSMUX2_MUX1_S       4
#define ADC_SSMUX2_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL2 register.
//
//*****************************************************************************
#define ADC_SSCTL2_TS3          0x00008000  // 4th Sample Temp Sensor Select
#define ADC_SSCTL2_IE3          0x00004000  // 4th Sample Interrupt Enable
#define ADC_SSCTL2_END3         0x00002000  // 4th Sample is End of Sequence
#define ADC_SSCTL2_D3           0x00001000  // 4th Sample Differential Input
                                            // Select
#define ADC_SSCTL2_TS2          0x00000800  // 3rd Sample Temp Sensor Select
#define ADC_SSCTL2_IE2          0x00000400  // 3rd Sample Interrupt Enable
#define ADC_SSCTL2_END2         0x00000200  // 3rd Sample is End of Sequence
#define ADC_SSCTL2_D2           0x00000100  // 3rd Sample Differential Input
                                            // Select
#define ADC_SSCTL2_TS1          0x00000080  // 2nd Sample Temp Sensor Select
#define ADC_SSCTL2_IE1          0x00000040  // 2nd Sample Interrupt Enable
#define ADC_SSCTL2_END1         0x00000020  // 2nd Sample is End of Sequence
#define ADC_SSCTL2_D1           0x00000010  // 2nd Sample Differential Input
                                            // Select
#define ADC_SSCTL2_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL2_IE0          0x00000004  // 1st Sample Interrupt Enable
#define ADC_SSCTL2_END0         0x00000002  // 1st Sample is End of Sequence
#define ADC_SSCTL2_D0           0x00000001  // 1st Sample Differential Input
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO2 register.
//
//*****************************************************************************
#define ADC_SSFIFO2_DATA_M      0x00000FFF  // Conversion Result Data
#define ADC_SSFIFO2_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT2 register.
//
//*****************************************************************************
#define ADC_SSFSTAT2_FULL       0x00001000  // FIFO Full
#define ADC_SSFSTAT2_EMPTY      0x00000100  // FIFO Empty
#define ADC_SSFSTAT2_HPTR_M     0x000000F0  // FIFO Head Pointer
#define ADC_SSFSTAT2_TPTR_M     0x0000000F  // FIFO Tail Pointer
#define ADC_SSFSTAT2_HPTR_S     4
#define ADC_SSFSTAT2_TPTR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP2 register.
//
//*****************************************************************************
#define ADC_SSOP2_S3DCOP        0x00001000  // Sample 3 Digital Comparator
                                            // Operation
#define ADC_SSOP2_S2DCOP        0x00000100  // Sample 2 Digital Comparator
                                            // Operation
#define ADC_SSOP2_S1DCOP        0x00000010  // Sample 1 Digital Comparator
                                            // Operation
#define ADC_SSOP2_S0DCOP        0x00000001  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC2 register.
//
//*****************************************************************************
#define ADC_SSDC2_S3DCSEL_M     0x0000F000  // Sample 3 Digital Comparator
                                            // Select
#define ADC_SSDC2_S2DCSEL_M     0x00000F00  // Sample 2 Digital Comparator
                                            // Select
#define ADC_SSDC2_S1DCSEL_M     0x000000F0  // Sample 1 Digital Comparator
                                            // Select
#define ADC_SSDC2_S0DCSEL_M     0x0000000F  // Sample 0 Digital Comparator
                                            // Select
#define ADC_SSDC2_S2DCSEL_S     8
#define ADC_SSDC2_S1DCSEL_S     4
#define ADC_SSDC2_S0DCSEL_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX3 register.
//
//*****************************************************************************
#define ADC_SSMUX3_MUX0_M       0x0000000F  // 1st Sample Input Select
#define ADC_SSMUX3_MUX0_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL3 register.
//
//*****************************************************************************
#define ADC_SSCTL3_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL3_IE0          0x00000004  // Sample Interrupt Enable
#define ADC_SSCTL3_END0         0x00000002  // End of Sequence
#define ADC_SSCTL3_D0           0x00000001  // Sample Differential Input Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO3 register.
//
//*****************************************************************************
#define ADC_SSFIFO3_DATA_M      0x00000FFF  // Conversion Result Data
#define ADC_SSFIFO3_DATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT3 register.
//
//*****************************************************************************
#define ADC_SSFSTAT3_FULL       0x00001000  // FIFO Full
#define ADC_SSFSTAT3_EMPTY      0x00000100  // FIFO Empty
#define ADC_SSFSTAT3_HPTR_M     0x000000F0  // FIFO Head Pointer
#define ADC_SSFSTAT3_TPTR_M     0x0000000F  // FIFO Tail Pointer
#define ADC_SSFSTAT3_HPTR_S     4
#define ADC_SSFSTAT3_TPTR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP3 register.
//
//*****************************************************************************
#define ADC_SSOP3_S0DCOP        0x00000001  // Sample 0 Digital Comparator
                                            // Operation

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC3 register.
//
//*****************************************************************************
#define ADC_SSDC3_S0DCSEL_M     0x0000000F  // Sample 0 Digital Comparator
                                            // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCRIC register.
//
//*****************************************************************************
#define ADC_DCRIC_DCTRIG7       0x00800000  // Digital Comparator Trigger 7
#define ADC_DCRIC_DCTRIG6       0x00400000  // Digital Comparator Trigger 6
#define ADC_DCRIC_DCTRIG5       0x00200000  // Digital Comparator Trigger 5
#define ADC_DCRIC_DCTRIG4       0x00100000  // Digital Comparator Trigger 4
#define ADC_DCRIC_DCTRIG3       0x00080000  // Digital Comparator Trigger 3
#define ADC_DCRIC_DCTRIG2       0x00040000  // Digital Comparator Trigger 2
#define ADC_DCRIC_DCTRIG1       0x00020000  // Digital Comparator Trigger 1
#define ADC_DCRIC_DCTRIG0       0x00010000  // Digital Comparator Trigger 0
#define ADC_DCRIC_DCINT7        0x00000080  // Digital Comparator Interrupt 7
#define ADC_DCRIC_DCINT6        0x00000040  // Digital Comparator Interrupt 6
#define ADC_DCRIC_DCINT5        0x00000020  // Digital Comparator Interrupt 5
#define ADC_DCRIC_DCINT4        0x00000010  // Digital Comparator Interrupt 4
#define ADC_DCRIC_DCINT3        0x00000008  // Digital Comparator Interrupt 3
#define ADC_DCRIC_DCINT2        0x00000004  // Digital Comparator Interrupt 2
#define ADC_DCRIC_DCINT1        0x00000002  // Digital Comparator Interrupt 1
#define ADC_DCRIC_DCINT0        0x00000001  // Digital Comparator Interrupt 0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL0 register.
//
//*****************************************************************************
#define ADC_DCCTL0_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL0_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL0_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL0_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL0_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL0_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL0_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL0_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL0_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL0_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL0_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL0_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL0_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL0_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL0_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL0_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL0_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL0_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL0_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL0_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL1 register.
//
//*****************************************************************************
#define ADC_DCCTL1_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL1_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL1_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL1_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL1_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL1_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL1_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL1_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL1_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL1_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL1_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL1_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL1_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL1_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL1_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL1_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL1_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL1_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL1_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL1_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL2 register.
//
//*****************************************************************************
#define ADC_DCCTL2_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL2_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL2_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL2_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL2_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL2_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL2_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL2_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL2_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL2_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL2_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL2_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL2_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL2_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL2_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL2_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL2_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL2_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL2_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL2_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL3 register.
//
//*****************************************************************************
#define ADC_DCCTL3_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL3_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL3_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL3_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL3_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL3_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL3_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL3_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL3_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL3_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL3_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL3_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL3_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL3_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL3_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL3_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL3_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL3_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL3_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL3_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL4 register.
//
//*****************************************************************************
#define ADC_DCCTL4_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL4_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL4_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL4_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL4_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL4_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL4_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL4_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL4_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL4_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL4_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL4_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL4_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL4_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL4_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL4_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL4_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL4_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL4_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL4_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL5 register.
//
//*****************************************************************************
#define ADC_DCCTL5_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL5_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL5_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL5_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL5_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL5_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL5_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL5_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL5_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL5_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL5_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL5_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL5_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL5_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL5_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL5_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL5_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL5_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL5_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL5_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL6 register.
//
//*****************************************************************************
#define ADC_DCCTL6_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL6_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL6_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL6_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL6_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL6_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL6_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL6_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL6_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL6_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL6_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL6_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL6_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL6_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL6_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL6_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL6_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL6_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL6_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL6_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL7 register.
//
//*****************************************************************************
#define ADC_DCCTL7_CTE          0x00001000  // Comparison Trigger Enable
#define ADC_DCCTL7_CTC_M        0x00000C00  // Comparison Trigger Condition
#define ADC_DCCTL7_CTC_LOW      0x00000000  // Low Band
#define ADC_DCCTL7_CTC_MID      0x00000400  // Mid Band
#define ADC_DCCTL7_CTC_HIGH     0x00000C00  // High Band
#define ADC_DCCTL7_CTM_M        0x00000300  // Comparison Trigger Mode
#define ADC_DCCTL7_CTM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL7_CTM_ONCE     0x00000100  // Once
#define ADC_DCCTL7_CTM_HALWAYS  0x00000200  // Hysteresis Always
#define ADC_DCCTL7_CTM_HONCE    0x00000300  // Hysteresis Once
#define ADC_DCCTL7_CIE          0x00000010  // Comparison Interrupt Enable
#define ADC_DCCTL7_CIC_M        0x0000000C  // Comparison Interrupt Condition
#define ADC_DCCTL7_CIC_LOW      0x00000000  // Low Band
#define ADC_DCCTL7_CIC_MID      0x00000004  // Mid Band
#define ADC_DCCTL7_CIC_HIGH     0x0000000C  // High Band
#define ADC_DCCTL7_CIM_M        0x00000003  // Comparison Interrupt Mode
#define ADC_DCCTL7_CIM_ALWAYS   0x00000000  // Always
#define ADC_DCCTL7_CIM_ONCE     0x00000001  // Once
#define ADC_DCCTL7_CIM_HALWAYS  0x00000002  // Hysteresis Always
#define ADC_DCCTL7_CIM_HONCE    0x00000003  // Hysteresis Once

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP0 register.
//
//*****************************************************************************
#define ADC_DCCMP0_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP0_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP0_COMP1_S      16
#define ADC_DCCMP0_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP1 register.
//
//*****************************************************************************
#define ADC_DCCMP1_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP1_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP1_COMP1_S      16
#define ADC_DCCMP1_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP2 register.
//
//*****************************************************************************
#define ADC_DCCMP2_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP2_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP2_COMP1_S      16
#define ADC_DCCMP2_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP3 register.
//
//*****************************************************************************
#define ADC_DCCMP3_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP3_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP3_COMP1_S      16
#define ADC_DCCMP3_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP4 register.
//
//*****************************************************************************
#define ADC_DCCMP4_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP4_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP4_COMP1_S      16
#define ADC_DCCMP4_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP5 register.
//
//*****************************************************************************
#define ADC_DCCMP5_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP5_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP5_COMP1_S      16
#define ADC_DCCMP5_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP6 register.
//
//*****************************************************************************
#define ADC_DCCMP6_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP6_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP6_COMP1_S      16
#define ADC_DCCMP6_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP7 register.
//
//*****************************************************************************
#define ADC_DCCMP7_COMP1_M      0x0FFF0000  // Compare 1
#define ADC_DCCMP7_COMP0_M      0x00000FFF  // Compare 0
#define ADC_DCCMP7_COMP1_S      16
#define ADC_DCCMP7_COMP0_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PP register.
//
//*****************************************************************************
#define ADC_PP_TS               0x00800000  // Temperature Sensor
#define ADC_PP_RSL_M            0x007C0000  // Resolution
#define ADC_PP_TYPE_M           0x00030000  // ADC Architecture
#define ADC_PP_TYPE_SAR         0x00000000  // SAR
#define ADC_PP_DC_M             0x0000FC00  // Digital Comparator Count
#define ADC_PP_CH_M             0x000003F0  // ADC Channel Count
#define ADC_PP_MSR_M            0x0000000F  // Maximum ADC Sample Rate
#define ADC_PP_MSR_125K         0x00000001  // 125 ksps
#define ADC_PP_MSR_250K         0x00000003  // 250 ksps
#define ADC_PP_MSR_500K         0x00000005  // 500 ksps
#define ADC_PP_MSR_1M           0x00000007  // 1 Msps
#define ADC_PP_RSL_S            18
#define ADC_PP_DC_S             10
#define ADC_PP_CH_S             4

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PC register.
//
//*****************************************************************************
#define ADC_PC_SR_M             0x0000000F  // ADC Sample Rate
#define ADC_PC_SR_125K          0x00000001  // 125 ksps
#define ADC_PC_SR_250K          0x00000003  // 250 ksps
#define ADC_PC_SR_500K          0x00000005  // 500 ksps
#define ADC_PC_SR_1M            0x00000007  // 1 Msps

//*****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CC register.
//
//*****************************************************************************
#define ADC_CC_CS_M             0x0000000F  // ADC Clock Source
#define ADC_CC_CS_SYSPLL        0x00000000  // PLL VCO divided by CLKDIV
#define ADC_CC_CS_PIOSC         0x00000001  // PIOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACMIS register.
//
//*****************************************************************************
#define COMP_ACMIS_IN1          0x00000002  // Comparator 1 Masked Interrupt
                                            // Status
#define COMP_ACMIS_IN0          0x00000001  // Comparator 0 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACRIS register.
//
//*****************************************************************************
#define COMP_ACRIS_IN1          0x00000002  // Comparator 1 Interrupt Status
#define COMP_ACRIS_IN0          0x00000001  // Comparator 0 Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACINTEN register.
//
//*****************************************************************************
#define COMP_ACINTEN_IN1        0x00000002  // Comparator 1 Interrupt Enable
#define COMP_ACINTEN_IN0        0x00000001  // Comparator 0 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACREFCTL
// register.
//
//*****************************************************************************
#define COMP_ACREFCTL_EN        0x00000200  // Resistor Ladder Enable
#define COMP_ACREFCTL_RNG       0x00000100  // Resistor Ladder Range
#define COMP_ACREFCTL_VREF_M    0x0000000F  // Resistor Ladder Voltage Ref
#define COMP_ACREFCTL_VREF_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT0 register.
//
//*****************************************************************************
#define COMP_ACSTAT0_OVAL       0x00000002  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL0 register.
//
//*****************************************************************************
#define COMP_ACCTL0_TOEN        0x00000800  // Trigger Output Enable
#define COMP_ACCTL0_ASRCP_M     0x00000600  // Analog Source Positive
#define COMP_ACCTL0_ASRCP_PIN   0x00000000  // Pin value of Cn+
#define COMP_ACCTL0_ASRCP_PIN0  0x00000200  // Pin value of C0+
#define COMP_ACCTL0_ASRCP_REF   0x00000400  // Internal voltage reference
#define COMP_ACCTL0_TSLVAL      0x00000080  // Trigger Sense Level Value
#define COMP_ACCTL0_TSEN_M      0x00000060  // Trigger Sense
#define COMP_ACCTL0_TSEN_LEVEL  0x00000000  // Level sense, see TSLVAL
#define COMP_ACCTL0_TSEN_FALL   0x00000020  // Falling edge
#define COMP_ACCTL0_TSEN_RISE   0x00000040  // Rising edge
#define COMP_ACCTL0_TSEN_BOTH   0x00000060  // Either edge
#define COMP_ACCTL0_ISLVAL      0x00000010  // Interrupt Sense Level Value
#define COMP_ACCTL0_ISEN_M      0x0000000C  // Interrupt Sense
#define COMP_ACCTL0_ISEN_LEVEL  0x00000000  // Level sense, see ISLVAL
#define COMP_ACCTL0_ISEN_FALL   0x00000004  // Falling edge
#define COMP_ACCTL0_ISEN_RISE   0x00000008  // Rising edge
#define COMP_ACCTL0_ISEN_BOTH   0x0000000C  // Either edge
#define COMP_ACCTL0_CINV        0x00000002  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT1 register.
//
//*****************************************************************************
#define COMP_ACSTAT1_OVAL       0x00000002  // Comparator Output Value

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL1 register.
//
//*****************************************************************************
#define COMP_ACCTL1_TOEN        0x00000800  // Trigger Output Enable
#define COMP_ACCTL1_ASRCP_M     0x00000600  // Analog Source Positive
#define COMP_ACCTL1_ASRCP_PIN   0x00000000  // Pin value of Cn+
#define COMP_ACCTL1_ASRCP_PIN0  0x00000200  // Pin value of C0+
#define COMP_ACCTL1_ASRCP_REF   0x00000400  // Internal voltage reference
#define COMP_ACCTL1_TSLVAL      0x00000080  // Trigger Sense Level Value
#define COMP_ACCTL1_TSEN_M      0x00000060  // Trigger Sense
#define COMP_ACCTL1_TSEN_LEVEL  0x00000000  // Level sense, see TSLVAL
#define COMP_ACCTL1_TSEN_FALL   0x00000020  // Falling edge
#define COMP_ACCTL1_TSEN_RISE   0x00000040  // Rising edge
#define COMP_ACCTL1_TSEN_BOTH   0x00000060  // Either edge
#define COMP_ACCTL1_ISLVAL      0x00000010  // Interrupt Sense Level Value
#define COMP_ACCTL1_ISEN_M      0x0000000C  // Interrupt Sense
#define COMP_ACCTL1_ISEN_LEVEL  0x00000000  // Level sense, see ISLVAL
#define COMP_ACCTL1_ISEN_FALL   0x00000004  // Falling edge
#define COMP_ACCTL1_ISEN_RISE   0x00000008  // Rising edge
#define COMP_ACCTL1_ISEN_BOTH   0x0000000C  // Either edge
#define COMP_ACCTL1_CINV        0x00000002  // Comparator Output Invert

//*****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_PP register.
//
//*****************************************************************************
#define COMP_PP_C1O             0x00020000  // Comparator Output 1 Present
#define COMP_PP_C0O             0x00010000  // Comparator Output 0 Present
#define COMP_PP_CMP1            0x00000002  // Comparator 1 Present
#define COMP_PP_CMP0            0x00000001  // Comparator 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_CTL register.
//
//*****************************************************************************
#define CAN_CTL_TEST            0x00000080  // Test Mode Enable
#define CAN_CTL_CCE             0x00000040  // Configuration Change Enable
#define CAN_CTL_DAR             0x00000020  // Disable Automatic-Retransmission
#define CAN_CTL_EIE             0x00000008  // Error Interrupt Enable
#define CAN_CTL_SIE             0x00000004  // Status Interrupt Enable
#define CAN_CTL_IE              0x00000002  // CAN Interrupt Enable
#define CAN_CTL_INIT            0x00000001  // Initialization

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_STS register.
//
//*****************************************************************************
#define CAN_STS_BOFF            0x00000080  // Bus-Off Status
#define CAN_STS_EWARN           0x00000040  // Warning Status
#define CAN_STS_EPASS           0x00000020  // Error Passive
#define CAN_STS_RXOK            0x00000010  // Received a Message Successfully
#define CAN_STS_TXOK            0x00000008  // Transmitted a Message
                                            // Successfully
#define CAN_STS_LEC_M           0x00000007  // Last Error Code
#define CAN_STS_LEC_NONE        0x00000000  // No Error
#define CAN_STS_LEC_STUFF       0x00000001  // Stuff Error
#define CAN_STS_LEC_FORM        0x00000002  // Format Error
#define CAN_STS_LEC_ACK         0x00000003  // ACK Error
#define CAN_STS_LEC_BIT1        0x00000004  // Bit 1 Error
#define CAN_STS_LEC_BIT0        0x00000005  // Bit 0 Error
#define CAN_STS_LEC_CRC         0x00000006  // CRC Error
#define CAN_STS_LEC_NOEVENT     0x00000007  // No Event

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_ERR register.
//
//*****************************************************************************
#define CAN_ERR_RP              0x00008000  // Received Error Passive
#define CAN_ERR_REC_M           0x00007F00  // Receive Error Counter
#define CAN_ERR_TEC_M           0x000000FF  // Transmit Error Counter
#define CAN_ERR_REC_S           8
#define CAN_ERR_TEC_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BIT register.
//
//*****************************************************************************
#define CAN_BIT_TSEG2_M         0x00007000  // Time Segment after Sample Point
#define CAN_BIT_TSEG1_M         0x00000F00  // Time Segment Before Sample Point
#define CAN_BIT_SJW_M           0x000000C0  // (Re)Synchronization Jump Width
#define CAN_BIT_BRP_M           0x0000003F  // Baud Rate Prescaler
#define CAN_BIT_TSEG2_S         12
#define CAN_BIT_TSEG1_S         8
#define CAN_BIT_SJW_S           6
#define CAN_BIT_BRP_S           0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_INT register.
//
//*****************************************************************************
#define CAN_INT_INTID_M         0x0000FFFF  // Interrupt Identifier
#define CAN_INT_INTID_NONE      0x00000000  // No interrupt pending
#define CAN_INT_INTID_STATUS    0x00008000  // Status Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TST register.
//
//*****************************************************************************
#define CAN_TST_RX              0x00000080  // Receive Observation
#define CAN_TST_TX_M            0x00000060  // Transmit Control
#define CAN_TST_TX_CANCTL       0x00000000  // CAN Module Control
#define CAN_TST_TX_SAMPLE       0x00000020  // Sample Point
#define CAN_TST_TX_DOMINANT     0x00000040  // Driven Low
#define CAN_TST_TX_RECESSIVE    0x00000060  // Driven High
#define CAN_TST_LBACK           0x00000010  // Loopback Mode
#define CAN_TST_SILENT          0x00000008  // Silent Mode
#define CAN_TST_BASIC           0x00000004  // Basic Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BRPE register.
//
//*****************************************************************************
#define CAN_BRPE_BRPE_M         0x0000000F  // Baud Rate Prescaler Extension
#define CAN_BRPE_BRPE_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CRQ register.
//
//*****************************************************************************
#define CAN_IF1CRQ_BUSY         0x00008000  // Busy Flag
#define CAN_IF1CRQ_MNUM_M       0x0000003F  // Message Number
#define CAN_IF1CRQ_MNUM_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CMSK register.
//
//*****************************************************************************
#define CAN_IF1CMSK_WRNRD       0x00000080  // Write, Not Read
#define CAN_IF1CMSK_MASK        0x00000040  // Access Mask Bits
#define CAN_IF1CMSK_ARB         0x00000020  // Access Arbitration Bits
#define CAN_IF1CMSK_CONTROL     0x00000010  // Access Control Bits
#define CAN_IF1CMSK_CLRINTPND   0x00000008  // Clear Interrupt Pending Bit
#define CAN_IF1CMSK_NEWDAT      0x00000004  // Access New Data
#define CAN_IF1CMSK_TXRQST      0x00000004  // Access Transmission Request
#define CAN_IF1CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3
#define CAN_IF1CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK1 register.
//
//*****************************************************************************
#define CAN_IF1MSK1_IDMSK_M     0x0000FFFF  // Identifier Mask
#define CAN_IF1MSK1_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK2 register.
//
//*****************************************************************************
#define CAN_IF1MSK2_MXTD        0x00008000  // Mask Extended Identifier
#define CAN_IF1MSK2_MDIR        0x00004000  // Mask Message Direction
#define CAN_IF1MSK2_IDMSK_M     0x00001FFF  // Identifier Mask
#define CAN_IF1MSK2_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB1 register.
//
//*****************************************************************************
#define CAN_IF1ARB1_ID_M        0x0000FFFF  // Message Identifier
#define CAN_IF1ARB1_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB2 register.
//
//*****************************************************************************
#define CAN_IF1ARB2_MSGVAL      0x00008000  // Message Valid
#define CAN_IF1ARB2_XTD         0x00004000  // Extended Identifier
#define CAN_IF1ARB2_DIR         0x00002000  // Message Direction
#define CAN_IF1ARB2_ID_M        0x00001FFF  // Message Identifier
#define CAN_IF1ARB2_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MCTL register.
//
//*****************************************************************************
#define CAN_IF1MCTL_NEWDAT      0x00008000  // New Data
#define CAN_IF1MCTL_MSGLST      0x00004000  // Message Lost
#define CAN_IF1MCTL_INTPND      0x00002000  // Interrupt Pending
#define CAN_IF1MCTL_UMASK       0x00001000  // Use Acceptance Mask
#define CAN_IF1MCTL_TXIE        0x00000800  // Transmit Interrupt Enable
#define CAN_IF1MCTL_RXIE        0x00000400  // Receive Interrupt Enable
#define CAN_IF1MCTL_RMTEN       0x00000200  // Remote Enable
#define CAN_IF1MCTL_TXRQST      0x00000100  // Transmit Request
#define CAN_IF1MCTL_EOB         0x00000080  // End of Buffer
#define CAN_IF1MCTL_DLC_M       0x0000000F  // Data Length Code
#define CAN_IF1MCTL_DLC_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA1 register.
//
//*****************************************************************************
#define CAN_IF1DA1_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DA1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA2 register.
//
//*****************************************************************************
#define CAN_IF1DA2_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DA2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB1 register.
//
//*****************************************************************************
#define CAN_IF1DB1_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DB1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB2 register.
//
//*****************************************************************************
#define CAN_IF1DB2_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DB2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CRQ register.
//
//*****************************************************************************
#define CAN_IF2CRQ_BUSY         0x00008000  // Busy Flag
#define CAN_IF2CRQ_MNUM_M       0x0000003F  // Message Number
#define CAN_IF2CRQ_MNUM_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CMSK register.
//
//*****************************************************************************
#define CAN_IF2CMSK_WRNRD       0x00000080  // Write, Not Read
#define CAN_IF2CMSK_MASK        0x00000040  // Access Mask Bits
#define CAN_IF2CMSK_ARB         0x00000020  // Access Arbitration Bits
#define CAN_IF2CMSK_CONTROL     0x00000010  // Access Control Bits
#define CAN_IF2CMSK_CLRINTPND   0x00000008  // Clear Interrupt Pending Bit
#define CAN_IF2CMSK_NEWDAT      0x00000004  // Access New Data
#define CAN_IF2CMSK_TXRQST      0x00000004  // Access Transmission Request
#define CAN_IF2CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3
#define CAN_IF2CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK1 register.
//
//*****************************************************************************
#define CAN_IF2MSK1_IDMSK_M     0x0000FFFF  // Identifier Mask
#define CAN_IF2MSK1_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK2 register.
//
//*****************************************************************************
#define CAN_IF2MSK2_MXTD        0x00008000  // Mask Extended Identifier
#define CAN_IF2MSK2_MDIR        0x00004000  // Mask Message Direction
#define CAN_IF2MSK2_IDMSK_M     0x00001FFF  // Identifier Mask
#define CAN_IF2MSK2_IDMSK_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB1 register.
//
//*****************************************************************************
#define CAN_IF2ARB1_ID_M        0x0000FFFF  // Message Identifier
#define CAN_IF2ARB1_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB2 register.
//
//*****************************************************************************
#define CAN_IF2ARB2_MSGVAL      0x00008000  // Message Valid
#define CAN_IF2ARB2_XTD         0x00004000  // Extended Identifier
#define CAN_IF2ARB2_DIR         0x00002000  // Message Direction
#define CAN_IF2ARB2_ID_M        0x00001FFF  // Message Identifier
#define CAN_IF2ARB2_ID_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MCTL register.
//
//*****************************************************************************
#define CAN_IF2MCTL_NEWDAT      0x00008000  // New Data
#define CAN_IF2MCTL_MSGLST      0x00004000  // Message Lost
#define CAN_IF2MCTL_INTPND      0x00002000  // Interrupt Pending
#define CAN_IF2MCTL_UMASK       0x00001000  // Use Acceptance Mask
#define CAN_IF2MCTL_TXIE        0x00000800  // Transmit Interrupt Enable
#define CAN_IF2MCTL_RXIE        0x00000400  // Receive Interrupt Enable
#define CAN_IF2MCTL_RMTEN       0x00000200  // Remote Enable
#define CAN_IF2MCTL_TXRQST      0x00000100  // Transmit Request
#define CAN_IF2MCTL_EOB         0x00000080  // End of Buffer
#define CAN_IF2MCTL_DLC_M       0x0000000F  // Data Length Code
#define CAN_IF2MCTL_DLC_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA1 register.
//
//*****************************************************************************
#define CAN_IF2DA1_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DA1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA2 register.
//
//*****************************************************************************
#define CAN_IF2DA2_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DA2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB1 register.
//
//*****************************************************************************
#define CAN_IF2DB1_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DB1_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB2 register.
//
//*****************************************************************************
#define CAN_IF2DB2_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DB2_DATA_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ1 register.
//
//*****************************************************************************
#define CAN_TXRQ1_TXRQST_M      0x0000FFFF  // Transmission Request Bits
#define CAN_TXRQ1_TXRQST_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ2 register.
//
//*****************************************************************************
#define CAN_TXRQ2_TXRQST_M      0x0000FFFF  // Transmission Request Bits
#define CAN_TXRQ2_TXRQST_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA1 register.
//
//*****************************************************************************
#define CAN_NWDA1_NEWDAT_M      0x0000FFFF  // New Data Bits
#define CAN_NWDA1_NEWDAT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA2 register.
//
//*****************************************************************************
#define CAN_NWDA2_NEWDAT_M      0x0000FFFF  // New Data Bits
#define CAN_NWDA2_NEWDAT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1INT register.
//
//*****************************************************************************
#define CAN_MSG1INT_INTPND_M    0x0000FFFF  // Interrupt Pending Bits
#define CAN_MSG1INT_INTPND_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2INT register.
//
//*****************************************************************************
#define CAN_MSG2INT_INTPND_M    0x0000FFFF  // Interrupt Pending Bits
#define CAN_MSG2INT_INTPND_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1VAL register.
//
//*****************************************************************************
#define CAN_MSG1VAL_MSGVAL_M    0x0000FFFF  // Message Valid Bits
#define CAN_MSG1VAL_MSGVAL_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2VAL register.
//
//*****************************************************************************
#define CAN_MSG2VAL_MSGVAL_M    0x0000FFFF  // Message Valid Bits
#define CAN_MSG2VAL_MSGVAL_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FADDR register.
//
//*****************************************************************************
#define USB_FADDR_M             0x0000007F  // Function Address
#define USB_FADDR_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_POWER register.
//
//*****************************************************************************
#define USB_POWER_ISOUP         0x00000080  // Isochronous Update
#define USB_POWER_SOFTCONN      0x00000040  // Soft Connect/Disconnect
#define USB_POWER_RESET         0x00000008  // RESET Signaling
#define USB_POWER_RESUME        0x00000004  // RESUME Signaling
#define USB_POWER_SUSPEND       0x00000002  // SUSPEND Mode
#define USB_POWER_PWRDNPHY      0x00000001  // Power Down PHY

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIS register.
//
//*****************************************************************************
#define USB_TXIS_EP7            0x00000080  // TX Endpoint 7 Interrupt
#define USB_TXIS_EP6            0x00000040  // TX Endpoint 6 Interrupt
#define USB_TXIS_EP5            0x00000020  // TX Endpoint 5 Interrupt
#define USB_TXIS_EP4            0x00000010  // TX Endpoint 4 Interrupt
#define USB_TXIS_EP3            0x00000008  // TX Endpoint 3 Interrupt
#define USB_TXIS_EP2            0x00000004  // TX Endpoint 2 Interrupt
#define USB_TXIS_EP1            0x00000002  // TX Endpoint 1 Interrupt
#define USB_TXIS_EP0            0x00000001  // TX and RX Endpoint 0 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIS register.
//
//*****************************************************************************
#define USB_RXIS_EP7            0x00000080  // RX Endpoint 7 Interrupt
#define USB_RXIS_EP6            0x00000040  // RX Endpoint 6 Interrupt
#define USB_RXIS_EP5            0x00000020  // RX Endpoint 5 Interrupt
#define USB_RXIS_EP4            0x00000010  // RX Endpoint 4 Interrupt
#define USB_RXIS_EP3            0x00000008  // RX Endpoint 3 Interrupt
#define USB_RXIS_EP2            0x00000004  // RX Endpoint 2 Interrupt
#define USB_RXIS_EP1            0x00000002  // RX Endpoint 1 Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIE register.
//
//*****************************************************************************
#define USB_TXIE_EP7            0x00000080  // TX Endpoint 7 Interrupt Enable
#define USB_TXIE_EP6            0x00000040  // TX Endpoint 6 Interrupt Enable
#define USB_TXIE_EP5            0x00000020  // TX Endpoint 5 Interrupt Enable
#define USB_TXIE_EP4            0x00000010  // TX Endpoint 4 Interrupt Enable
#define USB_TXIE_EP3            0x00000008  // TX Endpoint 3 Interrupt Enable
#define USB_TXIE_EP2            0x00000004  // TX Endpoint 2 Interrupt Enable
#define USB_TXIE_EP1            0x00000002  // TX Endpoint 1 Interrupt Enable
#define USB_TXIE_EP0            0x00000001  // TX and RX Endpoint 0 Interrupt
                                            // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIE register.
//
//*****************************************************************************
#define USB_RXIE_EP7            0x00000080  // RX Endpoint 7 Interrupt Enable
#define USB_RXIE_EP6            0x00000040  // RX Endpoint 6 Interrupt Enable
#define USB_RXIE_EP5            0x00000020  // RX Endpoint 5 Interrupt Enable
#define USB_RXIE_EP4            0x00000010  // RX Endpoint 4 Interrupt Enable
#define USB_RXIE_EP3            0x00000008  // RX Endpoint 3 Interrupt Enable
#define USB_RXIE_EP2            0x00000004  // RX Endpoint 2 Interrupt Enable
#define USB_RXIE_EP1            0x00000002  // RX Endpoint 1 Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IS register.
//
//*****************************************************************************
#define USB_IS_VBUSERR          0x00000080  // VBUS Error (OTG only)
#define USB_IS_SESREQ           0x00000040  // SESSION REQUEST (OTG only)
#define USB_IS_DISCON           0x00000020  // Session Disconnect (OTG only)
#define USB_IS_CONN             0x00000010  // Session Connect
#define USB_IS_SOF              0x00000008  // Start of Frame
#define USB_IS_BABBLE           0x00000004  // Babble Detected
#define USB_IS_RESET            0x00000004  // RESET Signaling Detected
#define USB_IS_RESUME           0x00000002  // RESUME Signaling Detected
#define USB_IS_SUSPEND          0x00000001  // SUSPEND Signaling Detected

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IE register.
//
//*****************************************************************************
#define USB_IE_VBUSERR          0x00000080  // Enable VBUS Error Interrupt (OTG
                                            // only)
#define USB_IE_SESREQ           0x00000040  // Enable Session Request (OTG
                                            // only)
#define USB_IE_DISCON           0x00000020  // Enable Disconnect Interrupt
#define USB_IE_CONN             0x00000010  // Enable Connect Interrupt
#define USB_IE_SOF              0x00000008  // Enable Start-of-Frame Interrupt
#define USB_IE_BABBLE           0x00000004  // Enable Babble Interrupt
#define USB_IE_RESET            0x00000004  // Enable RESET Interrupt
#define USB_IE_RESUME           0x00000002  // Enable RESUME Interrupt
#define USB_IE_SUSPND           0x00000001  // Enable SUSPEND Interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FRAME register.
//
//*****************************************************************************
#define USB_FRAME_M             0x000007FF  // Frame Number
#define USB_FRAME_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPIDX register.
//
//*****************************************************************************
#define USB_EPIDX_EPIDX_M       0x0000000F  // Endpoint Index
#define USB_EPIDX_EPIDX_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TEST register.
//
//*****************************************************************************
#define USB_TEST_FORCEH         0x00000080  // Force Host Mode
#define USB_TEST_FIFOACC        0x00000040  // FIFO Access
#define USB_TEST_FORCEFS        0x00000020  // Force Full-Speed Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO0 register.
//
//*****************************************************************************
#define USB_FIFO0_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO0_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO1 register.
//
//*****************************************************************************
#define USB_FIFO1_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO1_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO2 register.
//
//*****************************************************************************
#define USB_FIFO2_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO2_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO3 register.
//
//*****************************************************************************
#define USB_FIFO3_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO3_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO4 register.
//
//*****************************************************************************
#define USB_FIFO4_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO4_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO5 register.
//
//*****************************************************************************
#define USB_FIFO5_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO5_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO6 register.
//
//*****************************************************************************
#define USB_FIFO6_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO6_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO7 register.
//
//*****************************************************************************
#define USB_FIFO7_EPDATA_M      0xFFFFFFFF  // Endpoint Data
#define USB_FIFO7_EPDATA_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DEVCTL register.
//
//*****************************************************************************
#define USB_DEVCTL_DEV          0x00000080  // Device Mode (OTG only)
#define USB_DEVCTL_FSDEV        0x00000040  // Full-Speed Device Detected
#define USB_DEVCTL_LSDEV        0x00000020  // Low-Speed Device Detected
#define USB_DEVCTL_VBUS_M       0x00000018  // VBUS Level (OTG only)
#define USB_DEVCTL_VBUS_NONE    0x00000000  // Below SessionEnd
#define USB_DEVCTL_VBUS_SEND    0x00000008  // Above SessionEnd, below AValid
#define USB_DEVCTL_VBUS_AVALID  0x00000010  // Above AValid, below VBUSValid
#define USB_DEVCTL_VBUS_VALID   0x00000018  // Above VBUSValid
#define USB_DEVCTL_HOST         0x00000004  // Host Mode
#define USB_DEVCTL_HOSTREQ      0x00000002  // Host Request (OTG only)
#define USB_DEVCTL_SESSION      0x00000001  // Session Start/End (OTG only)

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOSZ register.
//
//*****************************************************************************
#define USB_TXFIFOSZ_DPB        0x00000010  // Double Packet Buffer Support
#define USB_TXFIFOSZ_SIZE_M     0x0000000F  // Max Packet Size
#define USB_TXFIFOSZ_SIZE_8     0x00000000  // 8
#define USB_TXFIFOSZ_SIZE_16    0x00000001  // 16
#define USB_TXFIFOSZ_SIZE_32    0x00000002  // 32
#define USB_TXFIFOSZ_SIZE_64    0x00000003  // 64
#define USB_TXFIFOSZ_SIZE_128   0x00000004  // 128
#define USB_TXFIFOSZ_SIZE_256   0x00000005  // 256
#define USB_TXFIFOSZ_SIZE_512   0x00000006  // 512
#define USB_TXFIFOSZ_SIZE_1024  0x00000007  // 1024
#define USB_TXFIFOSZ_SIZE_2048  0x00000008  // 2048

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOSZ register.
//
//*****************************************************************************
#define USB_RXFIFOSZ_DPB        0x00000010  // Double Packet Buffer Support
#define USB_RXFIFOSZ_SIZE_M     0x0000000F  // Max Packet Size
#define USB_RXFIFOSZ_SIZE_8     0x00000000  // 8
#define USB_RXFIFOSZ_SIZE_16    0x00000001  // 16
#define USB_RXFIFOSZ_SIZE_32    0x00000002  // 32
#define USB_RXFIFOSZ_SIZE_64    0x00000003  // 64
#define USB_RXFIFOSZ_SIZE_128   0x00000004  // 128
#define USB_RXFIFOSZ_SIZE_256   0x00000005  // 256
#define USB_RXFIFOSZ_SIZE_512   0x00000006  // 512
#define USB_RXFIFOSZ_SIZE_1024  0x00000007  // 1024
#define USB_RXFIFOSZ_SIZE_2048  0x00000008  // 2048

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOADD
// register.
//
//*****************************************************************************
#define USB_TXFIFOADD_ADDR_M    0x000001FF  // Transmit/Receive Start Address
#define USB_TXFIFOADD_ADDR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOADD
// register.
//
//*****************************************************************************
#define USB_RXFIFOADD_ADDR_M    0x000001FF  // Transmit/Receive Start Address
#define USB_RXFIFOADD_ADDR_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CONTIM register.
//
//*****************************************************************************
#define USB_CONTIM_WTCON_M      0x000000F0  // Connect Wait
#define USB_CONTIM_WTID_M       0x0000000F  // Wait ID
#define USB_CONTIM_WTCON_S      4
#define USB_CONTIM_WTID_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VPLEN register.
//
//*****************************************************************************
#define USB_VPLEN_VPLEN_M       0x000000FF  // VBUS Pulse Length
#define USB_VPLEN_VPLEN_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FSEOF register.
//
//*****************************************************************************
#define USB_FSEOF_FSEOFG_M      0x000000FF  // Full-Speed End-of-Frame Gap
#define USB_FSEOF_FSEOFG_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LSEOF register.
//
//*****************************************************************************
#define USB_LSEOF_LSEOFG_M      0x000000FF  // Low-Speed End-of-Frame Gap
#define USB_LSEOF_LSEOFG_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR0
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR0_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR0_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR0
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR0_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR0_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT0
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT0_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT0_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR1
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR1_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR1_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR1
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR1_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR1_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT1
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT1_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT1_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR1
// register.
//
//*****************************************************************************
#define USB_RXFUNCADDR1_ADDR_M  0x0000007F  // Device Address
#define USB_RXFUNCADDR1_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR1
// register.
//
//*****************************************************************************
#define USB_RXHUBADDR1_ADDR_M   0x0000007F  // Hub Address
#define USB_RXHUBADDR1_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT1
// register.
//
//*****************************************************************************
#define USB_RXHUBPORT1_PORT_M   0x0000007F  // Hub Port
#define USB_RXHUBPORT1_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR2
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR2_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR2_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR2
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR2_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR2_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT2
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT2_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT2_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR2
// register.
//
//*****************************************************************************
#define USB_RXFUNCADDR2_ADDR_M  0x0000007F  // Device Address
#define USB_RXFUNCADDR2_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR2
// register.
//
//*****************************************************************************
#define USB_RXHUBADDR2_ADDR_M   0x0000007F  // Hub Address
#define USB_RXHUBADDR2_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT2
// register.
//
//*****************************************************************************
#define USB_RXHUBPORT2_PORT_M   0x0000007F  // Hub Port
#define USB_RXHUBPORT2_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR3
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR3_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR3_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR3
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR3_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR3_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT3
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT3_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT3_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR3
// register.
//
//*****************************************************************************
#define USB_RXFUNCADDR3_ADDR_M  0x0000007F  // Device Address
#define USB_RXFUNCADDR3_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR3
// register.
//
//*****************************************************************************
#define USB_RXHUBADDR3_ADDR_M   0x0000007F  // Hub Address
#define USB_RXHUBADDR3_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT3
// register.
//
//*****************************************************************************
#define USB_RXHUBPORT3_PORT_M   0x0000007F  // Hub Port
#define USB_RXHUBPORT3_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR4
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR4_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR4_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR4
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR4_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR4_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT4
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT4_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT4_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR4
// register.
//
//*****************************************************************************
#define USB_RXFUNCADDR4_ADDR_M  0x0000007F  // Device Address
#define USB_RXFUNCADDR4_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR4
// register.
//
//*****************************************************************************
#define USB_RXHUBADDR4_ADDR_M   0x0000007F  // Hub Address
#define USB_RXHUBADDR4_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT4
// register.
//
//*****************************************************************************
#define USB_RXHUBPORT4_PORT_M   0x0000007F  // Hub Port
#define USB_RXHUBPORT4_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR5
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR5_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR5_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR5
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR5_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR5_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT5
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT5_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT5_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR5
// register.
//
//*****************************************************************************
#define USB_RXFUNCADDR5_ADDR_M  0x0000007F  // Device Address
#define USB_RXFUNCADDR5_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR5
// register.
//
//*****************************************************************************
#define USB_RXHUBADDR5_ADDR_M   0x0000007F  // Hub Address
#define USB_RXHUBADDR5_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT5
// register.
//
//*****************************************************************************
#define USB_RXHUBPORT5_PORT_M   0x0000007F  // Hub Port
#define USB_RXHUBPORT5_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR6
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR6_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR6_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR6
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR6_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR6_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT6
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT6_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT6_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR6
// register.
//
//*****************************************************************************
#define USB_RXFUNCADDR6_ADDR_M  0x0000007F  // Device Address
#define USB_RXFUNCADDR6_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR6
// register.
//
//*****************************************************************************
#define USB_RXHUBADDR6_ADDR_M   0x0000007F  // Hub Address
#define USB_RXHUBADDR6_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT6
// register.
//
//*****************************************************************************
#define USB_RXHUBPORT6_PORT_M   0x0000007F  // Hub Port
#define USB_RXHUBPORT6_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFUNCADDR7
// register.
//
//*****************************************************************************
#define USB_TXFUNCADDR7_ADDR_M  0x0000007F  // Device Address
#define USB_TXFUNCADDR7_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBADDR7
// register.
//
//*****************************************************************************
#define USB_TXHUBADDR7_ADDR_M   0x0000007F  // Hub Address
#define USB_TXHUBADDR7_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXHUBPORT7
// register.
//
//*****************************************************************************
#define USB_TXHUBPORT7_PORT_M   0x0000007F  // Hub Port
#define USB_TXHUBPORT7_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFUNCADDR7
// register.
//
//*****************************************************************************
#define USB_RXFUNCADDR7_ADDR_M  0x0000007F  // Device Address
#define USB_RXFUNCADDR7_ADDR_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBADDR7
// register.
//
//*****************************************************************************
#define USB_RXHUBADDR7_ADDR_M   0x0000007F  // Hub Address
#define USB_RXHUBADDR7_ADDR_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXHUBPORT7
// register.
//
//*****************************************************************************
#define USB_RXHUBPORT7_PORT_M   0x0000007F  // Hub Port
#define USB_RXHUBPORT7_PORT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRL0 register.
//
//*****************************************************************************
#define USB_CSRL0_NAKTO         0x00000080  // NAK Timeout
#define USB_CSRL0_SETENDC       0x00000080  // Setup End Clear
#define USB_CSRL0_STATUS        0x00000040  // STATUS Packet
#define USB_CSRL0_RXRDYC        0x00000040  // RXRDY Clear
#define USB_CSRL0_REQPKT        0x00000020  // Request Packet
#define USB_CSRL0_STALL         0x00000020  // Send Stall
#define USB_CSRL0_SETEND        0x00000010  // Setup End
#define USB_CSRL0_ERROR         0x00000010  // Error
#define USB_CSRL0_DATAEND       0x00000008  // Data End
#define USB_CSRL0_SETUP         0x00000008  // Setup Packet
#define USB_CSRL0_STALLED       0x00000004  // Endpoint Stalled
#define USB_CSRL0_TXRDY         0x00000002  // Transmit Packet Ready
#define USB_CSRL0_RXRDY         0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRH0 register.
//
//*****************************************************************************
#define USB_CSRH0_DTWE          0x00000004  // Data Toggle Write Enable
#define USB_CSRH0_DT            0x00000002  // Data Toggle
#define USB_CSRH0_FLUSH         0x00000001  // Flush FIFO

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_COUNT0 register.
//
//*****************************************************************************
#define USB_COUNT0_COUNT_M      0x0000007F  // FIFO Count
#define USB_COUNT0_COUNT_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TYPE0 register.
//
//*****************************************************************************
#define USB_TYPE0_SPEED_M       0x000000C0  // Operating Speed
#define USB_TYPE0_SPEED_FULL    0x00000080  // Full
#define USB_TYPE0_SPEED_LOW     0x000000C0  // Low

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_NAKLMT register.
//
//*****************************************************************************
#define USB_NAKLMT_NAKLMT_M     0x0000001F  // EP0 NAK Limit
#define USB_NAKLMT_NAKLMT_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP1 register.
//
//*****************************************************************************
#define USB_TXMAXP1_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_TXMAXP1_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL1 register.
//
//*****************************************************************************
#define USB_TXCSRL1_NAKTO       0x00000080  // NAK Timeout
#define USB_TXCSRL1_CLRDT       0x00000040  // Clear Data Toggle
#define USB_TXCSRL1_STALLED     0x00000020  // Endpoint Stalled
#define USB_TXCSRL1_STALL       0x00000010  // Send STALL
#define USB_TXCSRL1_SETUP       0x00000010  // Setup Packet
#define USB_TXCSRL1_FLUSH       0x00000008  // Flush FIFO
#define USB_TXCSRL1_ERROR       0x00000004  // Error
#define USB_TXCSRL1_UNDRN       0x00000004  // Underrun
#define USB_TXCSRL1_FIFONE      0x00000002  // FIFO Not Empty
#define USB_TXCSRL1_TXRDY       0x00000001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH1 register.
//
//*****************************************************************************
#define USB_TXCSRH1_AUTOSET     0x00000080  // Auto Set
#define USB_TXCSRH1_ISO         0x00000040  // Isochronous Transfers
#define USB_TXCSRH1_MODE        0x00000020  // Mode
#define USB_TXCSRH1_DMAEN       0x00000010  // DMA Request Enable
#define USB_TXCSRH1_FDT         0x00000008  // Force Data Toggle
#define USB_TXCSRH1_DMAMOD      0x00000004  // DMA Request Mode
#define USB_TXCSRH1_DTWE        0x00000002  // Data Toggle Write Enable
#define USB_TXCSRH1_DT          0x00000001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP1 register.
//
//*****************************************************************************
#define USB_RXMAXP1_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_RXMAXP1_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL1 register.
//
//*****************************************************************************
#define USB_RXCSRL1_CLRDT       0x00000080  // Clear Data Toggle
#define USB_RXCSRL1_STALLED     0x00000040  // Endpoint Stalled
#define USB_RXCSRL1_STALL       0x00000020  // Send STALL
#define USB_RXCSRL1_REQPKT      0x00000020  // Request Packet
#define USB_RXCSRL1_FLUSH       0x00000010  // Flush FIFO
#define USB_RXCSRL1_DATAERR     0x00000008  // Data Error
#define USB_RXCSRL1_NAKTO       0x00000008  // NAK Timeout
#define USB_RXCSRL1_OVER        0x00000004  // Overrun
#define USB_RXCSRL1_ERROR       0x00000004  // Error
#define USB_RXCSRL1_FULL        0x00000002  // FIFO Full
#define USB_RXCSRL1_RXRDY       0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH1 register.
//
//*****************************************************************************
#define USB_RXCSRH1_AUTOCL      0x00000080  // Auto Clear
#define USB_RXCSRH1_AUTORQ      0x00000040  // Auto Request
#define USB_RXCSRH1_ISO         0x00000040  // Isochronous Transfers
#define USB_RXCSRH1_DMAEN       0x00000020  // DMA Request Enable
#define USB_RXCSRH1_DISNYET     0x00000010  // Disable NYET
#define USB_RXCSRH1_PIDERR      0x00000010  // PID Error
#define USB_RXCSRH1_DMAMOD      0x00000008  // DMA Request Mode
#define USB_RXCSRH1_DTWE        0x00000004  // Data Toggle Write Enable
#define USB_RXCSRH1_DT          0x00000002  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT1 register.
//
//*****************************************************************************
#define USB_RXCOUNT1_COUNT_M    0x00001FFF  // Receive Packet Count
#define USB_RXCOUNT1_COUNT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE1 register.
//
//*****************************************************************************
#define USB_TXTYPE1_SPEED_M     0x000000C0  // Operating Speed
#define USB_TXTYPE1_SPEED_DFLT  0x00000000  // Default
#define USB_TXTYPE1_SPEED_FULL  0x00000080  // Full
#define USB_TXTYPE1_SPEED_LOW   0x000000C0  // Low
#define USB_TXTYPE1_PROTO_M     0x00000030  // Protocol
#define USB_TXTYPE1_PROTO_CTRL  0x00000000  // Control
#define USB_TXTYPE1_PROTO_ISOC  0x00000010  // Isochronous
#define USB_TXTYPE1_PROTO_BULK  0x00000020  // Bulk
#define USB_TXTYPE1_PROTO_INT   0x00000030  // Interrupt
#define USB_TXTYPE1_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_TXTYPE1_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL1
// register.
//
//*****************************************************************************
#define USB_TXINTERVAL1_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_TXINTERVAL1_TXPOLL_M                                              \
                                0x000000FF  // TX Polling
#define USB_TXINTERVAL1_TXPOLL_S                                              \
                                0
#define USB_TXINTERVAL1_NAKLMT_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE1 register.
//
//*****************************************************************************
#define USB_RXTYPE1_SPEED_M     0x000000C0  // Operating Speed
#define USB_RXTYPE1_SPEED_DFLT  0x00000000  // Default
#define USB_RXTYPE1_SPEED_FULL  0x00000080  // Full
#define USB_RXTYPE1_SPEED_LOW   0x000000C0  // Low
#define USB_RXTYPE1_PROTO_M     0x00000030  // Protocol
#define USB_RXTYPE1_PROTO_CTRL  0x00000000  // Control
#define USB_RXTYPE1_PROTO_ISOC  0x00000010  // Isochronous
#define USB_RXTYPE1_PROTO_BULK  0x00000020  // Bulk
#define USB_RXTYPE1_PROTO_INT   0x00000030  // Interrupt
#define USB_RXTYPE1_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_RXTYPE1_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL1
// register.
//
//*****************************************************************************
#define USB_RXINTERVAL1_TXPOLL_M                                              \
                                0x000000FF  // RX Polling
#define USB_RXINTERVAL1_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_RXINTERVAL1_TXPOLL_S                                              \
                                0
#define USB_RXINTERVAL1_NAKLMT_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP2 register.
//
//*****************************************************************************
#define USB_TXMAXP2_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_TXMAXP2_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL2 register.
//
//*****************************************************************************
#define USB_TXCSRL2_NAKTO       0x00000080  // NAK Timeout
#define USB_TXCSRL2_CLRDT       0x00000040  // Clear Data Toggle
#define USB_TXCSRL2_STALLED     0x00000020  // Endpoint Stalled
#define USB_TXCSRL2_SETUP       0x00000010  // Setup Packet
#define USB_TXCSRL2_STALL       0x00000010  // Send STALL
#define USB_TXCSRL2_FLUSH       0x00000008  // Flush FIFO
#define USB_TXCSRL2_ERROR       0x00000004  // Error
#define USB_TXCSRL2_UNDRN       0x00000004  // Underrun
#define USB_TXCSRL2_FIFONE      0x00000002  // FIFO Not Empty
#define USB_TXCSRL2_TXRDY       0x00000001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH2 register.
//
//*****************************************************************************
#define USB_TXCSRH2_AUTOSET     0x00000080  // Auto Set
#define USB_TXCSRH2_ISO         0x00000040  // Isochronous Transfers
#define USB_TXCSRH2_MODE        0x00000020  // Mode
#define USB_TXCSRH2_DMAEN       0x00000010  // DMA Request Enable
#define USB_TXCSRH2_FDT         0x00000008  // Force Data Toggle
#define USB_TXCSRH2_DMAMOD      0x00000004  // DMA Request Mode
#define USB_TXCSRH2_DTWE        0x00000002  // Data Toggle Write Enable
#define USB_TXCSRH2_DT          0x00000001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP2 register.
//
//*****************************************************************************
#define USB_RXMAXP2_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_RXMAXP2_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL2 register.
//
//*****************************************************************************
#define USB_RXCSRL2_CLRDT       0x00000080  // Clear Data Toggle
#define USB_RXCSRL2_STALLED     0x00000040  // Endpoint Stalled
#define USB_RXCSRL2_REQPKT      0x00000020  // Request Packet
#define USB_RXCSRL2_STALL       0x00000020  // Send STALL
#define USB_RXCSRL2_FLUSH       0x00000010  // Flush FIFO
#define USB_RXCSRL2_DATAERR     0x00000008  // Data Error
#define USB_RXCSRL2_NAKTO       0x00000008  // NAK Timeout
#define USB_RXCSRL2_ERROR       0x00000004  // Error
#define USB_RXCSRL2_OVER        0x00000004  // Overrun
#define USB_RXCSRL2_FULL        0x00000002  // FIFO Full
#define USB_RXCSRL2_RXRDY       0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH2 register.
//
//*****************************************************************************
#define USB_RXCSRH2_AUTOCL      0x00000080  // Auto Clear
#define USB_RXCSRH2_AUTORQ      0x00000040  // Auto Request
#define USB_RXCSRH2_ISO         0x00000040  // Isochronous Transfers
#define USB_RXCSRH2_DMAEN       0x00000020  // DMA Request Enable
#define USB_RXCSRH2_DISNYET     0x00000010  // Disable NYET
#define USB_RXCSRH2_PIDERR      0x00000010  // PID Error
#define USB_RXCSRH2_DMAMOD      0x00000008  // DMA Request Mode
#define USB_RXCSRH2_DTWE        0x00000004  // Data Toggle Write Enable
#define USB_RXCSRH2_DT          0x00000002  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT2 register.
//
//*****************************************************************************
#define USB_RXCOUNT2_COUNT_M    0x00001FFF  // Receive Packet Count
#define USB_RXCOUNT2_COUNT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE2 register.
//
//*****************************************************************************
#define USB_TXTYPE2_SPEED_M     0x000000C0  // Operating Speed
#define USB_TXTYPE2_SPEED_DFLT  0x00000000  // Default
#define USB_TXTYPE2_SPEED_FULL  0x00000080  // Full
#define USB_TXTYPE2_SPEED_LOW   0x000000C0  // Low
#define USB_TXTYPE2_PROTO_M     0x00000030  // Protocol
#define USB_TXTYPE2_PROTO_CTRL  0x00000000  // Control
#define USB_TXTYPE2_PROTO_ISOC  0x00000010  // Isochronous
#define USB_TXTYPE2_PROTO_BULK  0x00000020  // Bulk
#define USB_TXTYPE2_PROTO_INT   0x00000030  // Interrupt
#define USB_TXTYPE2_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_TXTYPE2_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL2
// register.
//
//*****************************************************************************
#define USB_TXINTERVAL2_TXPOLL_M                                              \
                                0x000000FF  // TX Polling
#define USB_TXINTERVAL2_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_TXINTERVAL2_NAKLMT_S                                              \
                                0
#define USB_TXINTERVAL2_TXPOLL_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE2 register.
//
//*****************************************************************************
#define USB_RXTYPE2_SPEED_M     0x000000C0  // Operating Speed
#define USB_RXTYPE2_SPEED_DFLT  0x00000000  // Default
#define USB_RXTYPE2_SPEED_FULL  0x00000080  // Full
#define USB_RXTYPE2_SPEED_LOW   0x000000C0  // Low
#define USB_RXTYPE2_PROTO_M     0x00000030  // Protocol
#define USB_RXTYPE2_PROTO_CTRL  0x00000000  // Control
#define USB_RXTYPE2_PROTO_ISOC  0x00000010  // Isochronous
#define USB_RXTYPE2_PROTO_BULK  0x00000020  // Bulk
#define USB_RXTYPE2_PROTO_INT   0x00000030  // Interrupt
#define USB_RXTYPE2_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_RXTYPE2_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL2
// register.
//
//*****************************************************************************
#define USB_RXINTERVAL2_TXPOLL_M                                              \
                                0x000000FF  // RX Polling
#define USB_RXINTERVAL2_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_RXINTERVAL2_TXPOLL_S                                              \
                                0
#define USB_RXINTERVAL2_NAKLMT_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP3 register.
//
//*****************************************************************************
#define USB_TXMAXP3_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_TXMAXP3_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL3 register.
//
//*****************************************************************************
#define USB_TXCSRL3_NAKTO       0x00000080  // NAK Timeout
#define USB_TXCSRL3_CLRDT       0x00000040  // Clear Data Toggle
#define USB_TXCSRL3_STALLED     0x00000020  // Endpoint Stalled
#define USB_TXCSRL3_SETUP       0x00000010  // Setup Packet
#define USB_TXCSRL3_STALL       0x00000010  // Send STALL
#define USB_TXCSRL3_FLUSH       0x00000008  // Flush FIFO
#define USB_TXCSRL3_ERROR       0x00000004  // Error
#define USB_TXCSRL3_UNDRN       0x00000004  // Underrun
#define USB_TXCSRL3_FIFONE      0x00000002  // FIFO Not Empty
#define USB_TXCSRL3_TXRDY       0x00000001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH3 register.
//
//*****************************************************************************
#define USB_TXCSRH3_AUTOSET     0x00000080  // Auto Set
#define USB_TXCSRH3_ISO         0x00000040  // Isochronous Transfers
#define USB_TXCSRH3_MODE        0x00000020  // Mode
#define USB_TXCSRH3_DMAEN       0x00000010  // DMA Request Enable
#define USB_TXCSRH3_FDT         0x00000008  // Force Data Toggle
#define USB_TXCSRH3_DMAMOD      0x00000004  // DMA Request Mode
#define USB_TXCSRH3_DTWE        0x00000002  // Data Toggle Write Enable
#define USB_TXCSRH3_DT          0x00000001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP3 register.
//
//*****************************************************************************
#define USB_RXMAXP3_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_RXMAXP3_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL3 register.
//
//*****************************************************************************
#define USB_RXCSRL3_CLRDT       0x00000080  // Clear Data Toggle
#define USB_RXCSRL3_STALLED     0x00000040  // Endpoint Stalled
#define USB_RXCSRL3_STALL       0x00000020  // Send STALL
#define USB_RXCSRL3_REQPKT      0x00000020  // Request Packet
#define USB_RXCSRL3_FLUSH       0x00000010  // Flush FIFO
#define USB_RXCSRL3_DATAERR     0x00000008  // Data Error
#define USB_RXCSRL3_NAKTO       0x00000008  // NAK Timeout
#define USB_RXCSRL3_ERROR       0x00000004  // Error
#define USB_RXCSRL3_OVER        0x00000004  // Overrun
#define USB_RXCSRL3_FULL        0x00000002  // FIFO Full
#define USB_RXCSRL3_RXRDY       0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH3 register.
//
//*****************************************************************************
#define USB_RXCSRH3_AUTOCL      0x00000080  // Auto Clear
#define USB_RXCSRH3_AUTORQ      0x00000040  // Auto Request
#define USB_RXCSRH3_ISO         0x00000040  // Isochronous Transfers
#define USB_RXCSRH3_DMAEN       0x00000020  // DMA Request Enable
#define USB_RXCSRH3_DISNYET     0x00000010  // Disable NYET
#define USB_RXCSRH3_PIDERR      0x00000010  // PID Error
#define USB_RXCSRH3_DMAMOD      0x00000008  // DMA Request Mode
#define USB_RXCSRH3_DTWE        0x00000004  // Data Toggle Write Enable
#define USB_RXCSRH3_DT          0x00000002  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT3 register.
//
//*****************************************************************************
#define USB_RXCOUNT3_COUNT_M    0x00001FFF  // Receive Packet Count
#define USB_RXCOUNT3_COUNT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE3 register.
//
//*****************************************************************************
#define USB_TXTYPE3_SPEED_M     0x000000C0  // Operating Speed
#define USB_TXTYPE3_SPEED_DFLT  0x00000000  // Default
#define USB_TXTYPE3_SPEED_FULL  0x00000080  // Full
#define USB_TXTYPE3_SPEED_LOW   0x000000C0  // Low
#define USB_TXTYPE3_PROTO_M     0x00000030  // Protocol
#define USB_TXTYPE3_PROTO_CTRL  0x00000000  // Control
#define USB_TXTYPE3_PROTO_ISOC  0x00000010  // Isochronous
#define USB_TXTYPE3_PROTO_BULK  0x00000020  // Bulk
#define USB_TXTYPE3_PROTO_INT   0x00000030  // Interrupt
#define USB_TXTYPE3_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_TXTYPE3_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL3
// register.
//
//*****************************************************************************
#define USB_TXINTERVAL3_TXPOLL_M                                              \
                                0x000000FF  // TX Polling
#define USB_TXINTERVAL3_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_TXINTERVAL3_TXPOLL_S                                              \
                                0
#define USB_TXINTERVAL3_NAKLMT_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE3 register.
//
//*****************************************************************************
#define USB_RXTYPE3_SPEED_M     0x000000C0  // Operating Speed
#define USB_RXTYPE3_SPEED_DFLT  0x00000000  // Default
#define USB_RXTYPE3_SPEED_FULL  0x00000080  // Full
#define USB_RXTYPE3_SPEED_LOW   0x000000C0  // Low
#define USB_RXTYPE3_PROTO_M     0x00000030  // Protocol
#define USB_RXTYPE3_PROTO_CTRL  0x00000000  // Control
#define USB_RXTYPE3_PROTO_ISOC  0x00000010  // Isochronous
#define USB_RXTYPE3_PROTO_BULK  0x00000020  // Bulk
#define USB_RXTYPE3_PROTO_INT   0x00000030  // Interrupt
#define USB_RXTYPE3_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_RXTYPE3_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL3
// register.
//
//*****************************************************************************
#define USB_RXINTERVAL3_TXPOLL_M                                              \
                                0x000000FF  // RX Polling
#define USB_RXINTERVAL3_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_RXINTERVAL3_TXPOLL_S                                              \
                                0
#define USB_RXINTERVAL3_NAKLMT_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP4 register.
//
//*****************************************************************************
#define USB_TXMAXP4_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_TXMAXP4_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL4 register.
//
//*****************************************************************************
#define USB_TXCSRL4_NAKTO       0x00000080  // NAK Timeout
#define USB_TXCSRL4_CLRDT       0x00000040  // Clear Data Toggle
#define USB_TXCSRL4_STALLED     0x00000020  // Endpoint Stalled
#define USB_TXCSRL4_SETUP       0x00000010  // Setup Packet
#define USB_TXCSRL4_STALL       0x00000010  // Send STALL
#define USB_TXCSRL4_FLUSH       0x00000008  // Flush FIFO
#define USB_TXCSRL4_ERROR       0x00000004  // Error
#define USB_TXCSRL4_UNDRN       0x00000004  // Underrun
#define USB_TXCSRL4_FIFONE      0x00000002  // FIFO Not Empty
#define USB_TXCSRL4_TXRDY       0x00000001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH4 register.
//
//*****************************************************************************
#define USB_TXCSRH4_AUTOSET     0x00000080  // Auto Set
#define USB_TXCSRH4_ISO         0x00000040  // Isochronous Transfers
#define USB_TXCSRH4_MODE        0x00000020  // Mode
#define USB_TXCSRH4_DMAEN       0x00000010  // DMA Request Enable
#define USB_TXCSRH4_FDT         0x00000008  // Force Data Toggle
#define USB_TXCSRH4_DMAMOD      0x00000004  // DMA Request Mode
#define USB_TXCSRH4_DTWE        0x00000002  // Data Toggle Write Enable
#define USB_TXCSRH4_DT          0x00000001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP4 register.
//
//*****************************************************************************
#define USB_RXMAXP4_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_RXMAXP4_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL4 register.
//
//*****************************************************************************
#define USB_RXCSRL4_CLRDT       0x00000080  // Clear Data Toggle
#define USB_RXCSRL4_STALLED     0x00000040  // Endpoint Stalled
#define USB_RXCSRL4_STALL       0x00000020  // Send STALL
#define USB_RXCSRL4_REQPKT      0x00000020  // Request Packet
#define USB_RXCSRL4_FLUSH       0x00000010  // Flush FIFO
#define USB_RXCSRL4_NAKTO       0x00000008  // NAK Timeout
#define USB_RXCSRL4_DATAERR     0x00000008  // Data Error
#define USB_RXCSRL4_OVER        0x00000004  // Overrun
#define USB_RXCSRL4_ERROR       0x00000004  // Error
#define USB_RXCSRL4_FULL        0x00000002  // FIFO Full
#define USB_RXCSRL4_RXRDY       0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH4 register.
//
//*****************************************************************************
#define USB_RXCSRH4_AUTOCL      0x00000080  // Auto Clear
#define USB_RXCSRH4_AUTORQ      0x00000040  // Auto Request
#define USB_RXCSRH4_ISO         0x00000040  // Isochronous Transfers
#define USB_RXCSRH4_DMAEN       0x00000020  // DMA Request Enable
#define USB_RXCSRH4_DISNYET     0x00000010  // Disable NYET
#define USB_RXCSRH4_PIDERR      0x00000010  // PID Error
#define USB_RXCSRH4_DMAMOD      0x00000008  // DMA Request Mode
#define USB_RXCSRH4_DTWE        0x00000004  // Data Toggle Write Enable
#define USB_RXCSRH4_DT          0x00000002  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT4 register.
//
//*****************************************************************************
#define USB_RXCOUNT4_COUNT_M    0x00001FFF  // Receive Packet Count
#define USB_RXCOUNT4_COUNT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE4 register.
//
//*****************************************************************************
#define USB_TXTYPE4_SPEED_M     0x000000C0  // Operating Speed
#define USB_TXTYPE4_SPEED_DFLT  0x00000000  // Default
#define USB_TXTYPE4_SPEED_FULL  0x00000080  // Full
#define USB_TXTYPE4_SPEED_LOW   0x000000C0  // Low
#define USB_TXTYPE4_PROTO_M     0x00000030  // Protocol
#define USB_TXTYPE4_PROTO_CTRL  0x00000000  // Control
#define USB_TXTYPE4_PROTO_ISOC  0x00000010  // Isochronous
#define USB_TXTYPE4_PROTO_BULK  0x00000020  // Bulk
#define USB_TXTYPE4_PROTO_INT   0x00000030  // Interrupt
#define USB_TXTYPE4_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_TXTYPE4_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL4
// register.
//
//*****************************************************************************
#define USB_TXINTERVAL4_TXPOLL_M                                              \
                                0x000000FF  // TX Polling
#define USB_TXINTERVAL4_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_TXINTERVAL4_NAKLMT_S                                              \
                                0
#define USB_TXINTERVAL4_TXPOLL_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE4 register.
//
//*****************************************************************************
#define USB_RXTYPE4_SPEED_M     0x000000C0  // Operating Speed
#define USB_RXTYPE4_SPEED_DFLT  0x00000000  // Default
#define USB_RXTYPE4_SPEED_FULL  0x00000080  // Full
#define USB_RXTYPE4_SPEED_LOW   0x000000C0  // Low
#define USB_RXTYPE4_PROTO_M     0x00000030  // Protocol
#define USB_RXTYPE4_PROTO_CTRL  0x00000000  // Control
#define USB_RXTYPE4_PROTO_ISOC  0x00000010  // Isochronous
#define USB_RXTYPE4_PROTO_BULK  0x00000020  // Bulk
#define USB_RXTYPE4_PROTO_INT   0x00000030  // Interrupt
#define USB_RXTYPE4_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_RXTYPE4_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL4
// register.
//
//*****************************************************************************
#define USB_RXINTERVAL4_TXPOLL_M                                              \
                                0x000000FF  // RX Polling
#define USB_RXINTERVAL4_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_RXINTERVAL4_NAKLMT_S                                              \
                                0
#define USB_RXINTERVAL4_TXPOLL_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP5 register.
//
//*****************************************************************************
#define USB_TXMAXP5_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_TXMAXP5_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL5 register.
//
//*****************************************************************************
#define USB_TXCSRL5_NAKTO       0x00000080  // NAK Timeout
#define USB_TXCSRL5_CLRDT       0x00000040  // Clear Data Toggle
#define USB_TXCSRL5_STALLED     0x00000020  // Endpoint Stalled
#define USB_TXCSRL5_SETUP       0x00000010  // Setup Packet
#define USB_TXCSRL5_STALL       0x00000010  // Send STALL
#define USB_TXCSRL5_FLUSH       0x00000008  // Flush FIFO
#define USB_TXCSRL5_ERROR       0x00000004  // Error
#define USB_TXCSRL5_UNDRN       0x00000004  // Underrun
#define USB_TXCSRL5_FIFONE      0x00000002  // FIFO Not Empty
#define USB_TXCSRL5_TXRDY       0x00000001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH5 register.
//
//*****************************************************************************
#define USB_TXCSRH5_AUTOSET     0x00000080  // Auto Set
#define USB_TXCSRH5_ISO         0x00000040  // Isochronous Transfers
#define USB_TXCSRH5_MODE        0x00000020  // Mode
#define USB_TXCSRH5_DMAEN       0x00000010  // DMA Request Enable
#define USB_TXCSRH5_FDT         0x00000008  // Force Data Toggle
#define USB_TXCSRH5_DMAMOD      0x00000004  // DMA Request Mode
#define USB_TXCSRH5_DTWE        0x00000002  // Data Toggle Write Enable
#define USB_TXCSRH5_DT          0x00000001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP5 register.
//
//*****************************************************************************
#define USB_RXMAXP5_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_RXMAXP5_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL5 register.
//
//*****************************************************************************
#define USB_RXCSRL5_CLRDT       0x00000080  // Clear Data Toggle
#define USB_RXCSRL5_STALLED     0x00000040  // Endpoint Stalled
#define USB_RXCSRL5_STALL       0x00000020  // Send STALL
#define USB_RXCSRL5_REQPKT      0x00000020  // Request Packet
#define USB_RXCSRL5_FLUSH       0x00000010  // Flush FIFO
#define USB_RXCSRL5_NAKTO       0x00000008  // NAK Timeout
#define USB_RXCSRL5_DATAERR     0x00000008  // Data Error
#define USB_RXCSRL5_ERROR       0x00000004  // Error
#define USB_RXCSRL5_OVER        0x00000004  // Overrun
#define USB_RXCSRL5_FULL        0x00000002  // FIFO Full
#define USB_RXCSRL5_RXRDY       0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH5 register.
//
//*****************************************************************************
#define USB_RXCSRH5_AUTOCL      0x00000080  // Auto Clear
#define USB_RXCSRH5_AUTORQ      0x00000040  // Auto Request
#define USB_RXCSRH5_ISO         0x00000040  // Isochronous Transfers
#define USB_RXCSRH5_DMAEN       0x00000020  // DMA Request Enable
#define USB_RXCSRH5_DISNYET     0x00000010  // Disable NYET
#define USB_RXCSRH5_PIDERR      0x00000010  // PID Error
#define USB_RXCSRH5_DMAMOD      0x00000008  // DMA Request Mode
#define USB_RXCSRH5_DTWE        0x00000004  // Data Toggle Write Enable
#define USB_RXCSRH5_DT          0x00000002  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT5 register.
//
//*****************************************************************************
#define USB_RXCOUNT5_COUNT_M    0x00001FFF  // Receive Packet Count
#define USB_RXCOUNT5_COUNT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE5 register.
//
//*****************************************************************************
#define USB_TXTYPE5_SPEED_M     0x000000C0  // Operating Speed
#define USB_TXTYPE5_SPEED_DFLT  0x00000000  // Default
#define USB_TXTYPE5_SPEED_FULL  0x00000080  // Full
#define USB_TXTYPE5_SPEED_LOW   0x000000C0  // Low
#define USB_TXTYPE5_PROTO_M     0x00000030  // Protocol
#define USB_TXTYPE5_PROTO_CTRL  0x00000000  // Control
#define USB_TXTYPE5_PROTO_ISOC  0x00000010  // Isochronous
#define USB_TXTYPE5_PROTO_BULK  0x00000020  // Bulk
#define USB_TXTYPE5_PROTO_INT   0x00000030  // Interrupt
#define USB_TXTYPE5_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_TXTYPE5_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL5
// register.
//
//*****************************************************************************
#define USB_TXINTERVAL5_TXPOLL_M                                              \
                                0x000000FF  // TX Polling
#define USB_TXINTERVAL5_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_TXINTERVAL5_NAKLMT_S                                              \
                                0
#define USB_TXINTERVAL5_TXPOLL_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE5 register.
//
//*****************************************************************************
#define USB_RXTYPE5_SPEED_M     0x000000C0  // Operating Speed
#define USB_RXTYPE5_SPEED_DFLT  0x00000000  // Default
#define USB_RXTYPE5_SPEED_FULL  0x00000080  // Full
#define USB_RXTYPE5_SPEED_LOW   0x000000C0  // Low
#define USB_RXTYPE5_PROTO_M     0x00000030  // Protocol
#define USB_RXTYPE5_PROTO_CTRL  0x00000000  // Control
#define USB_RXTYPE5_PROTO_ISOC  0x00000010  // Isochronous
#define USB_RXTYPE5_PROTO_BULK  0x00000020  // Bulk
#define USB_RXTYPE5_PROTO_INT   0x00000030  // Interrupt
#define USB_RXTYPE5_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_RXTYPE5_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL5
// register.
//
//*****************************************************************************
#define USB_RXINTERVAL5_TXPOLL_M                                              \
                                0x000000FF  // RX Polling
#define USB_RXINTERVAL5_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_RXINTERVAL5_TXPOLL_S                                              \
                                0
#define USB_RXINTERVAL5_NAKLMT_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP6 register.
//
//*****************************************************************************
#define USB_TXMAXP6_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_TXMAXP6_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL6 register.
//
//*****************************************************************************
#define USB_TXCSRL6_NAKTO       0x00000080  // NAK Timeout
#define USB_TXCSRL6_CLRDT       0x00000040  // Clear Data Toggle
#define USB_TXCSRL6_STALLED     0x00000020  // Endpoint Stalled
#define USB_TXCSRL6_STALL       0x00000010  // Send STALL
#define USB_TXCSRL6_SETUP       0x00000010  // Setup Packet
#define USB_TXCSRL6_FLUSH       0x00000008  // Flush FIFO
#define USB_TXCSRL6_ERROR       0x00000004  // Error
#define USB_TXCSRL6_UNDRN       0x00000004  // Underrun
#define USB_TXCSRL6_FIFONE      0x00000002  // FIFO Not Empty
#define USB_TXCSRL6_TXRDY       0x00000001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH6 register.
//
//*****************************************************************************
#define USB_TXCSRH6_AUTOSET     0x00000080  // Auto Set
#define USB_TXCSRH6_ISO         0x00000040  // Isochronous Transfers
#define USB_TXCSRH6_MODE        0x00000020  // Mode
#define USB_TXCSRH6_DMAEN       0x00000010  // DMA Request Enable
#define USB_TXCSRH6_FDT         0x00000008  // Force Data Toggle
#define USB_TXCSRH6_DMAMOD      0x00000004  // DMA Request Mode
#define USB_TXCSRH6_DTWE        0x00000002  // Data Toggle Write Enable
#define USB_TXCSRH6_DT          0x00000001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP6 register.
//
//*****************************************************************************
#define USB_RXMAXP6_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_RXMAXP6_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL6 register.
//
//*****************************************************************************
#define USB_RXCSRL6_CLRDT       0x00000080  // Clear Data Toggle
#define USB_RXCSRL6_STALLED     0x00000040  // Endpoint Stalled
#define USB_RXCSRL6_REQPKT      0x00000020  // Request Packet
#define USB_RXCSRL6_STALL       0x00000020  // Send STALL
#define USB_RXCSRL6_FLUSH       0x00000010  // Flush FIFO
#define USB_RXCSRL6_NAKTO       0x00000008  // NAK Timeout
#define USB_RXCSRL6_DATAERR     0x00000008  // Data Error
#define USB_RXCSRL6_ERROR       0x00000004  // Error
#define USB_RXCSRL6_OVER        0x00000004  // Overrun
#define USB_RXCSRL6_FULL        0x00000002  // FIFO Full
#define USB_RXCSRL6_RXRDY       0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH6 register.
//
//*****************************************************************************
#define USB_RXCSRH6_AUTOCL      0x00000080  // Auto Clear
#define USB_RXCSRH6_AUTORQ      0x00000040  // Auto Request
#define USB_RXCSRH6_ISO         0x00000040  // Isochronous Transfers
#define USB_RXCSRH6_DMAEN       0x00000020  // DMA Request Enable
#define USB_RXCSRH6_DISNYET     0x00000010  // Disable NYET
#define USB_RXCSRH6_PIDERR      0x00000010  // PID Error
#define USB_RXCSRH6_DMAMOD      0x00000008  // DMA Request Mode
#define USB_RXCSRH6_DTWE        0x00000004  // Data Toggle Write Enable
#define USB_RXCSRH6_DT          0x00000002  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT6 register.
//
//*****************************************************************************
#define USB_RXCOUNT6_COUNT_M    0x00001FFF  // Receive Packet Count
#define USB_RXCOUNT6_COUNT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE6 register.
//
//*****************************************************************************
#define USB_TXTYPE6_SPEED_M     0x000000C0  // Operating Speed
#define USB_TXTYPE6_SPEED_DFLT  0x00000000  // Default
#define USB_TXTYPE6_SPEED_FULL  0x00000080  // Full
#define USB_TXTYPE6_SPEED_LOW   0x000000C0  // Low
#define USB_TXTYPE6_PROTO_M     0x00000030  // Protocol
#define USB_TXTYPE6_PROTO_CTRL  0x00000000  // Control
#define USB_TXTYPE6_PROTO_ISOC  0x00000010  // Isochronous
#define USB_TXTYPE6_PROTO_BULK  0x00000020  // Bulk
#define USB_TXTYPE6_PROTO_INT   0x00000030  // Interrupt
#define USB_TXTYPE6_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_TXTYPE6_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL6
// register.
//
//*****************************************************************************
#define USB_TXINTERVAL6_TXPOLL_M                                              \
                                0x000000FF  // TX Polling
#define USB_TXINTERVAL6_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_TXINTERVAL6_TXPOLL_S                                              \
                                0
#define USB_TXINTERVAL6_NAKLMT_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE6 register.
//
//*****************************************************************************
#define USB_RXTYPE6_SPEED_M     0x000000C0  // Operating Speed
#define USB_RXTYPE6_SPEED_DFLT  0x00000000  // Default
#define USB_RXTYPE6_SPEED_FULL  0x00000080  // Full
#define USB_RXTYPE6_SPEED_LOW   0x000000C0  // Low
#define USB_RXTYPE6_PROTO_M     0x00000030  // Protocol
#define USB_RXTYPE6_PROTO_CTRL  0x00000000  // Control
#define USB_RXTYPE6_PROTO_ISOC  0x00000010  // Isochronous
#define USB_RXTYPE6_PROTO_BULK  0x00000020  // Bulk
#define USB_RXTYPE6_PROTO_INT   0x00000030  // Interrupt
#define USB_RXTYPE6_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_RXTYPE6_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL6
// register.
//
//*****************************************************************************
#define USB_RXINTERVAL6_TXPOLL_M                                              \
                                0x000000FF  // RX Polling
#define USB_RXINTERVAL6_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_RXINTERVAL6_NAKLMT_S                                              \
                                0
#define USB_RXINTERVAL6_TXPOLL_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP7 register.
//
//*****************************************************************************
#define USB_TXMAXP7_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_TXMAXP7_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL7 register.
//
//*****************************************************************************
#define USB_TXCSRL7_NAKTO       0x00000080  // NAK Timeout
#define USB_TXCSRL7_CLRDT       0x00000040  // Clear Data Toggle
#define USB_TXCSRL7_STALLED     0x00000020  // Endpoint Stalled
#define USB_TXCSRL7_STALL       0x00000010  // Send STALL
#define USB_TXCSRL7_SETUP       0x00000010  // Setup Packet
#define USB_TXCSRL7_FLUSH       0x00000008  // Flush FIFO
#define USB_TXCSRL7_ERROR       0x00000004  // Error
#define USB_TXCSRL7_UNDRN       0x00000004  // Underrun
#define USB_TXCSRL7_FIFONE      0x00000002  // FIFO Not Empty
#define USB_TXCSRL7_TXRDY       0x00000001  // Transmit Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH7 register.
//
//*****************************************************************************
#define USB_TXCSRH7_AUTOSET     0x00000080  // Auto Set
#define USB_TXCSRH7_ISO         0x00000040  // Isochronous Transfers
#define USB_TXCSRH7_MODE        0x00000020  // Mode
#define USB_TXCSRH7_DMAEN       0x00000010  // DMA Request Enable
#define USB_TXCSRH7_FDT         0x00000008  // Force Data Toggle
#define USB_TXCSRH7_DMAMOD      0x00000004  // DMA Request Mode
#define USB_TXCSRH7_DTWE        0x00000002  // Data Toggle Write Enable
#define USB_TXCSRH7_DT          0x00000001  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP7 register.
//
//*****************************************************************************
#define USB_RXMAXP7_MAXLOAD_M   0x000007FF  // Maximum Payload
#define USB_RXMAXP7_MAXLOAD_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL7 register.
//
//*****************************************************************************
#define USB_RXCSRL7_CLRDT       0x00000080  // Clear Data Toggle
#define USB_RXCSRL7_STALLED     0x00000040  // Endpoint Stalled
#define USB_RXCSRL7_REQPKT      0x00000020  // Request Packet
#define USB_RXCSRL7_STALL       0x00000020  // Send STALL
#define USB_RXCSRL7_FLUSH       0x00000010  // Flush FIFO
#define USB_RXCSRL7_DATAERR     0x00000008  // Data Error
#define USB_RXCSRL7_NAKTO       0x00000008  // NAK Timeout
#define USB_RXCSRL7_ERROR       0x00000004  // Error
#define USB_RXCSRL7_OVER        0x00000004  // Overrun
#define USB_RXCSRL7_FULL        0x00000002  // FIFO Full
#define USB_RXCSRL7_RXRDY       0x00000001  // Receive Packet Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH7 register.
//
//*****************************************************************************
#define USB_RXCSRH7_AUTOCL      0x00000080  // Auto Clear
#define USB_RXCSRH7_ISO         0x00000040  // Isochronous Transfers
#define USB_RXCSRH7_AUTORQ      0x00000040  // Auto Request
#define USB_RXCSRH7_DMAEN       0x00000020  // DMA Request Enable
#define USB_RXCSRH7_PIDERR      0x00000010  // PID Error
#define USB_RXCSRH7_DISNYET     0x00000010  // Disable NYET
#define USB_RXCSRH7_DMAMOD      0x00000008  // DMA Request Mode
#define USB_RXCSRH7_DTWE        0x00000004  // Data Toggle Write Enable
#define USB_RXCSRH7_DT          0x00000002  // Data Toggle

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT7 register.
//
//*****************************************************************************
#define USB_RXCOUNT7_COUNT_M    0x00001FFF  // Receive Packet Count
#define USB_RXCOUNT7_COUNT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXTYPE7 register.
//
//*****************************************************************************
#define USB_TXTYPE7_SPEED_M     0x000000C0  // Operating Speed
#define USB_TXTYPE7_SPEED_DFLT  0x00000000  // Default
#define USB_TXTYPE7_SPEED_FULL  0x00000080  // Full
#define USB_TXTYPE7_SPEED_LOW   0x000000C0  // Low
#define USB_TXTYPE7_PROTO_M     0x00000030  // Protocol
#define USB_TXTYPE7_PROTO_CTRL  0x00000000  // Control
#define USB_TXTYPE7_PROTO_ISOC  0x00000010  // Isochronous
#define USB_TXTYPE7_PROTO_BULK  0x00000020  // Bulk
#define USB_TXTYPE7_PROTO_INT   0x00000030  // Interrupt
#define USB_TXTYPE7_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_TXTYPE7_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXINTERVAL7
// register.
//
//*****************************************************************************
#define USB_TXINTERVAL7_TXPOLL_M                                              \
                                0x000000FF  // TX Polling
#define USB_TXINTERVAL7_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_TXINTERVAL7_NAKLMT_S                                              \
                                0
#define USB_TXINTERVAL7_TXPOLL_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXTYPE7 register.
//
//*****************************************************************************
#define USB_RXTYPE7_SPEED_M     0x000000C0  // Operating Speed
#define USB_RXTYPE7_SPEED_DFLT  0x00000000  // Default
#define USB_RXTYPE7_SPEED_FULL  0x00000080  // Full
#define USB_RXTYPE7_SPEED_LOW   0x000000C0  // Low
#define USB_RXTYPE7_PROTO_M     0x00000030  // Protocol
#define USB_RXTYPE7_PROTO_CTRL  0x00000000  // Control
#define USB_RXTYPE7_PROTO_ISOC  0x00000010  // Isochronous
#define USB_RXTYPE7_PROTO_BULK  0x00000020  // Bulk
#define USB_RXTYPE7_PROTO_INT   0x00000030  // Interrupt
#define USB_RXTYPE7_TEP_M       0x0000000F  // Target Endpoint Number
#define USB_RXTYPE7_TEP_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXINTERVAL7
// register.
//
//*****************************************************************************
#define USB_RXINTERVAL7_TXPOLL_M                                              \
                                0x000000FF  // RX Polling
#define USB_RXINTERVAL7_NAKLMT_M                                              \
                                0x000000FF  // NAK Limit
#define USB_RXINTERVAL7_NAKLMT_S                                              \
                                0
#define USB_RXINTERVAL7_TXPOLL_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT1
// register.
//
//*****************************************************************************
#define USB_RQPKTCOUNT1_M       0x0000FFFF  // Block Transfer Packet Count
#define USB_RQPKTCOUNT1_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT2
// register.
//
//*****************************************************************************
#define USB_RQPKTCOUNT2_M       0x0000FFFF  // Block Transfer Packet Count
#define USB_RQPKTCOUNT2_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT3
// register.
//
//*****************************************************************************
#define USB_RQPKTCOUNT3_M       0x0000FFFF  // Block Transfer Packet Count
#define USB_RQPKTCOUNT3_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT4
// register.
//
//*****************************************************************************
#define USB_RQPKTCOUNT4_COUNT_M 0x0000FFFF  // Block Transfer Packet Count
#define USB_RQPKTCOUNT4_COUNT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT5
// register.
//
//*****************************************************************************
#define USB_RQPKTCOUNT5_COUNT_M 0x0000FFFF  // Block Transfer Packet Count
#define USB_RQPKTCOUNT5_COUNT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT6
// register.
//
//*****************************************************************************
#define USB_RQPKTCOUNT6_COUNT_M 0x0000FFFF  // Block Transfer Packet Count
#define USB_RQPKTCOUNT6_COUNT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RQPKTCOUNT7
// register.
//
//*****************************************************************************
#define USB_RQPKTCOUNT7_COUNT_M 0x0000FFFF  // Block Transfer Packet Count
#define USB_RQPKTCOUNT7_COUNT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXDPKTBUFDIS
// register.
//
//*****************************************************************************
#define USB_RXDPKTBUFDIS_EP7    0x00000080  // EP7 RX Double-Packet Buffer
                                            // Disable
#define USB_RXDPKTBUFDIS_EP6    0x00000040  // EP6 RX Double-Packet Buffer
                                            // Disable
#define USB_RXDPKTBUFDIS_EP5    0x00000020  // EP5 RX Double-Packet Buffer
                                            // Disable
#define USB_RXDPKTBUFDIS_EP4    0x00000010  // EP4 RX Double-Packet Buffer
                                            // Disable
#define USB_RXDPKTBUFDIS_EP3    0x00000008  // EP3 RX Double-Packet Buffer
                                            // Disable
#define USB_RXDPKTBUFDIS_EP2    0x00000004  // EP2 RX Double-Packet Buffer
                                            // Disable
#define USB_RXDPKTBUFDIS_EP1    0x00000002  // EP1 RX Double-Packet Buffer
                                            // Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXDPKTBUFDIS
// register.
//
//*****************************************************************************
#define USB_TXDPKTBUFDIS_EP7    0x00000080  // EP7 TX Double-Packet Buffer
                                            // Disable
#define USB_TXDPKTBUFDIS_EP6    0x00000040  // EP6 TX Double-Packet Buffer
                                            // Disable
#define USB_TXDPKTBUFDIS_EP5    0x00000020  // EP5 TX Double-Packet Buffer
                                            // Disable
#define USB_TXDPKTBUFDIS_EP4    0x00000010  // EP4 TX Double-Packet Buffer
                                            // Disable
#define USB_TXDPKTBUFDIS_EP3    0x00000008  // EP3 TX Double-Packet Buffer
                                            // Disable
#define USB_TXDPKTBUFDIS_EP2    0x00000004  // EP2 TX Double-Packet Buffer
                                            // Disable
#define USB_TXDPKTBUFDIS_EP1    0x00000002  // EP1 TX Double-Packet Buffer
                                            // Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPC register.
//
//*****************************************************************************
#define USB_EPC_PFLTACT_M       0x00000300  // Power Fault Action
#define USB_EPC_PFLTACT_UNCHG   0x00000000  // Unchanged
#define USB_EPC_PFLTACT_TRIS    0x00000100  // Tristate
#define USB_EPC_PFLTACT_LOW     0x00000200  // Low
#define USB_EPC_PFLTACT_HIGH    0x00000300  // High
#define USB_EPC_PFLTAEN         0x00000040  // Power Fault Action Enable
#define USB_EPC_PFLTSEN_HIGH    0x00000020  // Power Fault Sense
#define USB_EPC_PFLTEN          0x00000010  // Power Fault Input Enable
#define USB_EPC_EPENDE          0x00000004  // EPEN Drive Enable
#define USB_EPC_EPEN_M          0x00000003  // External Power Supply Enable
                                            // Configuration
#define USB_EPC_EPEN_LOW        0x00000000  // Power Enable Active Low
#define USB_EPC_EPEN_HIGH       0x00000001  // Power Enable Active High
#define USB_EPC_EPEN_VBLOW      0x00000002  // Power Enable High if VBUS Low
                                            // (OTG only)
#define USB_EPC_EPEN_VBHIGH     0x00000003  // Power Enable High if VBUS High
                                            // (OTG only)

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCRIS register.
//
//*****************************************************************************
#define USB_EPCRIS_PF           0x00000001  // USB Power Fault Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCIM register.
//
//*****************************************************************************
#define USB_EPCIM_PF            0x00000001  // USB Power Fault Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPCISC register.
//
//*****************************************************************************
#define USB_EPCISC_PF           0x00000001  // USB Power Fault Interrupt Status
                                            // and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRRIS register.
//
//*****************************************************************************
#define USB_DRRIS_RESUME        0x00000001  // RESUME Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRIM register.
//
//*****************************************************************************
#define USB_DRIM_RESUME         0x00000001  // RESUME Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRISC register.
//
//*****************************************************************************
#define USB_DRISC_RESUME        0x00000001  // RESUME Interrupt Status and
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_GPCS register.
//
//*****************************************************************************
#define USB_GPCS_DEVMODOTG      0x00000002  // Enable Device Mode
#define USB_GPCS_DEVMOD         0x00000001  // Device Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDC register.
//
//*****************************************************************************
#define USB_VDC_VBDEN           0x00000001  // VBUS Droop Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCRIS register.
//
//*****************************************************************************
#define USB_VDCRIS_VD           0x00000001  // VBUS Droop Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCIM register.
//
//*****************************************************************************
#define USB_VDCIM_VD            0x00000001  // VBUS Droop Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_VDCISC register.
//
//*****************************************************************************
#define USB_VDCISC_VD           0x00000001  // VBUS Droop Interrupt Status and
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVRIS register.
//
//*****************************************************************************
#define USB_IDVRIS_ID           0x00000001  // ID Valid Detect Raw Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVIM register.
//
//*****************************************************************************
#define USB_IDVIM_ID            0x00000001  // ID Valid Detect Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IDVISC register.
//
//*****************************************************************************
#define USB_IDVISC_ID           0x00000001  // ID Valid Detect Interrupt Status
                                            // and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMASEL register.
//
//*****************************************************************************
#define USB_DMASEL_DMACTX_M     0x00F00000  // DMA C TX Select
#define USB_DMASEL_DMACRX_M     0x000F0000  // DMA C RX Select
#define USB_DMASEL_DMABTX_M     0x0000F000  // DMA B TX Select
#define USB_DMASEL_DMABRX_M     0x00000F00  // DMA B RX Select
#define USB_DMASEL_DMAATX_M     0x000000F0  // DMA A TX Select
#define USB_DMASEL_DMAARX_M     0x0000000F  // DMA A RX Select
#define USB_DMASEL_DMACTX_S     20
#define USB_DMASEL_DMACRX_S     16
#define USB_DMASEL_DMABTX_S     12
#define USB_DMASEL_DMABRX_S     8
#define USB_DMASEL_DMAATX_S     4
#define USB_DMASEL_DMAARX_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the USB_O_PP register.
//
//*****************************************************************************
#define USB_PP_ECNT_M           0x0000FF00  // Endpoint Count
#define USB_PP_USB_M            0x000000C0  // USB Capability
#define USB_PP_USB_DEVICE       0x00000040  // DEVICE
#define USB_PP_USB_HOSTDEVICE   0x00000080  // HOST
#define USB_PP_USB_OTG          0x000000C0  // OTG
#define USB_PP_PHY              0x00000010  // PHY Present
#define USB_PP_TYPE_M           0x0000000F  // Controller Type
#define USB_PP_TYPE_0           0x00000000  // The first-generation USB
                                            // controller
#define USB_PP_ECNT_S           8

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESIZE register.
//
//*****************************************************************************
#define EEPROM_EESIZE_BLKCNT_M  0x07FF0000  // Number of 16-Word Blocks
#define EEPROM_EESIZE_WORDCNT_M 0x0000FFFF  // Number of 32-Bit Words
#define EEPROM_EESIZE_BLKCNT_S  16
#define EEPROM_EESIZE_WORDCNT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEBLOCK register.
//
//*****************************************************************************
#define EEPROM_EEBLOCK_BLOCK_M  0x0000FFFF  // Current Block
#define EEPROM_EEBLOCK_BLOCK_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEOFFSET
// register.
//
//*****************************************************************************
#define EEPROM_EEOFFSET_OFFSET_M                                              \
                                0x0000000F  // Current Address Offset
#define EEPROM_EEOFFSET_OFFSET_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWR register.
//
//*****************************************************************************
#define EEPROM_EERDWR_VALUE_M   0xFFFFFFFF  // EEPROM Read or Write Data
#define EEPROM_EERDWR_VALUE_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWRINC
// register.
//
//*****************************************************************************
#define EEPROM_EERDWRINC_VALUE_M                                              \
                                0xFFFFFFFF  // EEPROM Read or Write Data with
                                            // Increment
#define EEPROM_EERDWRINC_VALUE_S                                              \
                                0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDONE register.
//
//*****************************************************************************
#define EEPROM_EEDONE_WRBUSY    0x00000020  // Write Busy
#define EEPROM_EEDONE_NOPERM    0x00000010  // Write Without Permission
#define EEPROM_EEDONE_WKCOPY    0x00000008  // Working on a Copy
#define EEPROM_EEDONE_WKERASE   0x00000004  // Working on an Erase
#define EEPROM_EEDONE_WORKING   0x00000001  // EEPROM Working

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESUPP register.
//
//*****************************************************************************
#define EEPROM_EESUPP_PRETRY    0x00000008  // Programming Must Be Retried
#define EEPROM_EESUPP_ERETRY    0x00000004  // Erase Must Be Retried

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEUNLOCK
// register.
//
//*****************************************************************************
#define EEPROM_EEUNLOCK_UNLOCK_M                                              \
                                0xFFFFFFFF  // EEPROM Unlock

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPROT register.
//
//*****************************************************************************
#define EEPROM_EEPROT_ACC       0x00000008  // Access Control
#define EEPROM_EEPROT_PROT_M    0x00000007  // Protection Control
#define EEPROM_EEPROT_PROT_RWNPW                                              \
                                0x00000000  // This setting is the default. If
                                            // there is no password, the block
                                            // is not protected and is readable
                                            // and writable
#define EEPROM_EEPROT_PROT_RWPW 0x00000001  // If there is a password, the
                                            // block is readable or writable
                                            // only when unlocked
#define EEPROM_EEPROT_PROT_RONPW                                              \
                                0x00000002  // If there is no password, the
                                            // block is readable, not writable

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS0 register.
//
//*****************************************************************************
#define EEPROM_EEPASS0_PASS_M   0xFFFFFFFF  // Password
#define EEPROM_EEPASS0_PASS_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS1 register.
//
//*****************************************************************************
#define EEPROM_EEPASS1_PASS_M   0xFFFFFFFF  // Password
#define EEPROM_EEPASS1_PASS_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS2 register.
//
//*****************************************************************************
#define EEPROM_EEPASS2_PASS_M   0xFFFFFFFF  // Password
#define EEPROM_EEPASS2_PASS_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEINT register.
//
//*****************************************************************************
#define EEPROM_EEINT_INT        0x00000001  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE register.
//
//*****************************************************************************
#define EEPROM_EEHIDE_HN_M      0xFFFFFFFE  // Hide Block

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDBGME register.
//
//*****************************************************************************
#define EEPROM_EEDBGME_KEY_M    0xFFFF0000  // Erase Key
#define EEPROM_EEDBGME_ME       0x00000001  // Mass Erase
#define EEPROM_EEDBGME_KEY_S    16

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_PP register.
//
//*****************************************************************************
#define EEPROM_PP_SIZE_M        0x0000001F  // EEPROM Size
#define EEPROM_PP_SIZE_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_RIS register.
//
//*****************************************************************************
#define SYSEXC_RIS_FPIXCRIS     0x00000020  // Floating-Point Inexact Exception
                                            // Raw Interrupt Status
#define SYSEXC_RIS_FPOFCRIS     0x00000010  // Floating-Point Overflow
                                            // Exception Raw Interrupt Status
#define SYSEXC_RIS_FPUFCRIS     0x00000008  // Floating-Point Underflow
                                            // Exception Raw Interrupt Status
#define SYSEXC_RIS_FPIOCRIS     0x00000004  // Floating-Point Invalid Operation
                                            // Raw Interrupt Status
#define SYSEXC_RIS_FPDZCRIS     0x00000002  // Floating-Point Divide By 0
                                            // Exception Raw Interrupt Status
#define SYSEXC_RIS_FPIDCRIS     0x00000001  // Floating-Point Input Denormal
                                            // Exception Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IM register.
//
//*****************************************************************************
#define SYSEXC_IM_FPIXCIM       0x00000020  // Floating-Point Inexact Exception
                                            // Interrupt Mask
#define SYSEXC_IM_FPOFCIM       0x00000010  // Floating-Point Overflow
                                            // Exception Interrupt Mask
#define SYSEXC_IM_FPUFCIM       0x00000008  // Floating-Point Underflow
                                            // Exception Interrupt Mask
#define SYSEXC_IM_FPIOCIM       0x00000004  // Floating-Point Invalid Operation
                                            // Interrupt Mask
#define SYSEXC_IM_FPDZCIM       0x00000002  // Floating-Point Divide By 0
                                            // Exception Interrupt Mask
#define SYSEXC_IM_FPIDCIM       0x00000001  // Floating-Point Input Denormal
                                            // Exception Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_MIS register.
//
//*****************************************************************************
#define SYSEXC_MIS_FPIXCMIS     0x00000020  // Floating-Point Inexact Exception
                                            // Masked Interrupt Status
#define SYSEXC_MIS_FPOFCMIS     0x00000010  // Floating-Point Overflow
                                            // Exception Masked Interrupt
                                            // Status
#define SYSEXC_MIS_FPUFCMIS     0x00000008  // Floating-Point Underflow
                                            // Exception Masked Interrupt
                                            // Status
#define SYSEXC_MIS_FPIOCMIS     0x00000004  // Floating-Point Invalid Operation
                                            // Masked Interrupt Status
#define SYSEXC_MIS_FPDZCMIS     0x00000002  // Floating-Point Divide By 0
                                            // Exception Masked Interrupt
                                            // Status
#define SYSEXC_MIS_FPIDCMIS     0x00000001  // Floating-Point Input Denormal
                                            // Exception Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IC register.
//
//*****************************************************************************
#define SYSEXC_IC_FPIXCIC       0x00000020  // Floating-Point Inexact Exception
                                            // Interrupt Clear
#define SYSEXC_IC_FPOFCIC       0x00000010  // Floating-Point Overflow
                                            // Exception Interrupt Clear
#define SYSEXC_IC_FPUFCIC       0x00000008  // Floating-Point Underflow
                                            // Exception Interrupt Clear
#define SYSEXC_IC_FPIOCIC       0x00000004  // Floating-Point Invalid Operation
                                            // Interrupt Clear
#define SYSEXC_IC_FPDZCIC       0x00000002  // Floating-Point Divide By 0
                                            // Exception Interrupt Clear
#define SYSEXC_IC_FPIDCIC       0x00000001  // Floating-Point Input Denormal
                                            // Exception Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCC register.
//
//*****************************************************************************
#define HIB_RTCC_M              0xFFFFFFFF  // RTC Counter
#define HIB_RTCC_S              0

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCM0 register.
//
//*****************************************************************************
#define HIB_RTCM0_M             0xFFFFFFFF  // RTC Match 0
#define HIB_RTCM0_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCLD register.
//
//*****************************************************************************
#define HIB_RTCLD_M             0xFFFFFFFF  // RTC Load
#define HIB_RTCLD_S             0

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_CTL register.
//
//*****************************************************************************
#define HIB_CTL_WRC             0x80000000  // Write Complete/Capable
#define HIB_CTL_OSCDRV          0x00020000  // Oscillator Drive Capability
#define HIB_CTL_OSCBYP          0x00010000  // Oscillator Bypass
#define HIB_CTL_VBATSEL_M       0x00006000  // Select for Low-Battery
                                            // Comparator
#define HIB_CTL_VBATSEL_1_9V    0x00000000  // 1.9 Volts
#define HIB_CTL_VBATSEL_2_1V    0x00002000  // 2.1 Volts (default)
#define HIB_CTL_VBATSEL_2_3V    0x00004000  // 2.3 Volts
#define HIB_CTL_VBATSEL_2_5V    0x00006000  // 2.5 Volts
#define HIB_CTL_BATCHK          0x00000400  // Check Battery Status
#define HIB_CTL_BATWKEN         0x00000200  // Wake on Low Battery
#define HIB_CTL_VDD3ON          0x00000100  // VDD Powered
#define HIB_CTL_VABORT          0x00000080  // Power Cut Abort Enable
#define HIB_CTL_CLK32EN         0x00000040  // Clocking Enable
#define HIB_CTL_PINWEN          0x00000010  // External Wake Pin Enable
#define HIB_CTL_RTCWEN          0x00000008  // RTC Wake-up Enable
#define HIB_CTL_HIBREQ          0x00000002  // Hibernation Request
#define HIB_CTL_RTCEN           0x00000001  // RTC Timer Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IM register.
//
//*****************************************************************************
#define HIB_IM_WC               0x00000010  // External Write Complete/Capable
                                            // Interrupt Mask
#define HIB_IM_EXTW             0x00000008  // External Wake-Up Interrupt Mask
#define HIB_IM_LOWBAT           0x00000004  // Low Battery Voltage Interrupt
                                            // Mask
#define HIB_IM_RTCALT0          0x00000001  // RTC Alert 0 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RIS register.
//
//*****************************************************************************
#define HIB_RIS_WC              0x00000010  // Write Complete/Capable Raw
                                            // Interrupt Status
#define HIB_RIS_EXTW            0x00000008  // External Wake-Up Raw Interrupt
                                            // Status
#define HIB_RIS_LOWBAT          0x00000004  // Low Battery Voltage Raw
                                            // Interrupt Status
#define HIB_RIS_RTCALT0         0x00000001  // RTC Alert 0 Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_MIS register.
//
//*****************************************************************************
#define HIB_MIS_WC              0x00000010  // Write Complete/Capable Masked
                                            // Interrupt Status
#define HIB_MIS_EXTW            0x00000008  // External Wake-Up Masked
                                            // Interrupt Status
#define HIB_MIS_LOWBAT          0x00000004  // Low Battery Voltage Masked
                                            // Interrupt Status
#define HIB_MIS_RTCALT0         0x00000001  // RTC Alert 0 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_IC register.
//
//*****************************************************************************
#define HIB_IC_WC               0x00000010  // Write Complete/Capable Interrupt
                                            // Clear
#define HIB_IC_EXTW             0x00000008  // External Wake-Up Interrupt Clear
#define HIB_IC_LOWBAT           0x00000004  // Low Battery Voltage Interrupt
                                            // Clear
#define HIB_IC_RTCALT0          0x00000001  // RTC Alert0 Masked Interrupt
                                            // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCT register.
//
//*****************************************************************************
#define HIB_RTCT_TRIM_M         0x0000FFFF  // RTC Trim Value
#define HIB_RTCT_TRIM_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCSS register.
//
//*****************************************************************************
#define HIB_RTCSS_RTCSSM_M      0x7FFF0000  // RTC Sub Seconds Match
#define HIB_RTCSS_RTCSSC_M      0x00007FFF  // RTC Sub Seconds Count
#define HIB_RTCSS_RTCSSM_S      16
#define HIB_RTCSS_RTCSSC_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the HIB_DATA register.
//
//*****************************************************************************
#define HIB_DATA_RTD_M          0xFFFFFFFF  // Hibernation Module NV Data
#define HIB_DATA_RTD_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMA register.
//
//*****************************************************************************
#define FLASH_FMA_OFFSET_M      0x0003FFFF  // Address Offset
#define FLASH_FMA_OFFSET_S      0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMD register.
//
//*****************************************************************************
#define FLASH_FMD_DATA_M        0xFFFFFFFF  // Data Value
#define FLASH_FMD_DATA_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMC register.
//
//*****************************************************************************
#define FLASH_FMC_WRKEY         0xA4420000  // FLASH write key
#define FLASH_FMC_COMT          0x00000008  // Commit Register Value
#define FLASH_FMC_MERASE        0x00000004  // Mass Erase Flash Memory
#define FLASH_FMC_ERASE         0x00000002  // Erase a Page of Flash Memory
#define FLASH_FMC_WRITE         0x00000001  // Write a Word into Flash Memory

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCRIS register.
//
//*****************************************************************************
#define FLASH_FCRIS_PROGRIS     0x00002000  // Program Verify Error Raw
                                            // Interrupt Status
#define FLASH_FCRIS_ERRIS       0x00000800  // Erase Verify Error Raw Interrupt
                                            // Status
#define FLASH_FCRIS_INVDRIS     0x00000400  // Invalid Data Raw Interrupt
                                            // Status
#define FLASH_FCRIS_VOLTRIS     0x00000200  // Pump Voltage Raw Interrupt
                                            // Status
#define FLASH_FCRIS_ERIS        0x00000004  // EEPROM Raw Interrupt Status
#define FLASH_FCRIS_PRIS        0x00000002  // Programming Raw Interrupt Status
#define FLASH_FCRIS_ARIS        0x00000001  // Access Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCIM register.
//
//*****************************************************************************
#define FLASH_FCIM_PROGMASK     0x00002000  // PROGVER Interrupt Mask
#define FLASH_FCIM_ERMASK       0x00000800  // ERVER Interrupt Mask
#define FLASH_FCIM_INVDMASK     0x00000400  // Invalid Data Interrupt Mask
#define FLASH_FCIM_VOLTMASK     0x00000200  // VOLT Interrupt Mask
#define FLASH_FCIM_EMASK        0x00000004  // EEPROM Interrupt Mask
#define FLASH_FCIM_PMASK        0x00000002  // Programming Interrupt Mask
#define FLASH_FCIM_AMASK        0x00000001  // Access Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCMISC register.
//
//*****************************************************************************
#define FLASH_FCMISC_PROGMISC   0x00002000  // PROGVER Masked Interrupt Status
                                            // and Clear
#define FLASH_FCMISC_ERMISC     0x00000800  // ERVER Masked Interrupt Status
                                            // and Clear
#define FLASH_FCMISC_INVDMISC   0x00000400  // Invalid Data Masked Interrupt
                                            // Status and Clear
#define FLASH_FCMISC_VOLTMISC   0x00000200  // VOLT Masked Interrupt Status and
                                            // Clear
#define FLASH_FCMISC_EMISC      0x00000004  // EEPROM Masked Interrupt Status
                                            // and Clear
#define FLASH_FCMISC_PMISC      0x00000002  // Programming Masked Interrupt
                                            // Status and Clear
#define FLASH_FCMISC_AMISC      0x00000001  // Access Masked Interrupt Status
                                            // and Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMC2 register.
//
//*****************************************************************************
#define FLASH_FMC2_WRBUF        0x00000001  // Buffered Flash Memory Write

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FWBVAL register.
//
//*****************************************************************************
#define FLASH_FWBVAL_FWB_M      0xFFFFFFFF  // Flash Memory Write Buffer

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FWBN register.
//
//*****************************************************************************
#define FLASH_FWBN_DATA_M       0xFFFFFFFF  // Data

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FSIZE register.
//
//*****************************************************************************
#define FLASH_FSIZE_SIZE_M      0x0000FFFF  // Flash Size
#define FLASH_FSIZE_SIZE_256KB  0x0000007F  // 256 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_SSIZE register.
//
//*****************************************************************************
#define FLASH_SSIZE_SIZE_M      0x0000FFFF  // SRAM Size
#define FLASH_SSIZE_SIZE_32KB   0x0000007F  // 32 KB of SRAM

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_ROMSWMAP register.
//
//*****************************************************************************
#define FLASH_ROMSWMAP_SAFERTOS 0x00000001  // SafeRTOS Present

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_RMCTL register.
//
//*****************************************************************************
#define FLASH_RMCTL_BA          0x00000001  // Boot Alias

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_BOOTCFG register.
//
//*****************************************************************************
#define FLASH_BOOTCFG_NW        0x80000000  // Not Written
#define FLASH_BOOTCFG_PORT_M    0x0000E000  // Boot GPIO Port
#define FLASH_BOOTCFG_PORT_A    0x00000000  // Port A
#define FLASH_BOOTCFG_PORT_B    0x00002000  // Port B
#define FLASH_BOOTCFG_PORT_C    0x00004000  // Port C
#define FLASH_BOOTCFG_PORT_D    0x00006000  // Port D
#define FLASH_BOOTCFG_PORT_E    0x00008000  // Port E
#define FLASH_BOOTCFG_PORT_F    0x0000A000  // Port F
#define FLASH_BOOTCFG_PORT_G    0x0000C000  // Port G
#define FLASH_BOOTCFG_PORT_H    0x0000E000  // Port H
#define FLASH_BOOTCFG_PIN_M     0x00001C00  // Boot GPIO Pin
#define FLASH_BOOTCFG_PIN_0     0x00000000  // Pin 0
#define FLASH_BOOTCFG_PIN_1     0x00000400  // Pin 1
#define FLASH_BOOTCFG_PIN_2     0x00000800  // Pin 2
#define FLASH_BOOTCFG_PIN_3     0x00000C00  // Pin 3
#define FLASH_BOOTCFG_PIN_4     0x00001000  // Pin 4
#define FLASH_BOOTCFG_PIN_5     0x00001400  // Pin 5
#define FLASH_BOOTCFG_PIN_6     0x00001800  // Pin 6
#define FLASH_BOOTCFG_PIN_7     0x00001C00  // Pin 7
#define FLASH_BOOTCFG_POL       0x00000200  // Boot GPIO Polarity
#define FLASH_BOOTCFG_EN        0x00000100  // Boot GPIO Enable
#define FLASH_BOOTCFG_KEY       0x00000010  // KEY Select
#define FLASH_BOOTCFG_DBG1      0x00000002  // Debug Control 1
#define FLASH_BOOTCFG_DBG0      0x00000001  // Debug Control 0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG0 register.
//
//*****************************************************************************
#define FLASH_USERREG0_DATA_M   0xFFFFFFFF  // User Data
#define FLASH_USERREG0_DATA_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG1 register.
//
//*****************************************************************************
#define FLASH_USERREG1_DATA_M   0xFFFFFFFF  // User Data
#define FLASH_USERREG1_DATA_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG2 register.
//
//*****************************************************************************
#define FLASH_USERREG2_DATA_M   0xFFFFFFFF  // User Data
#define FLASH_USERREG2_DATA_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG3 register.
//
//*****************************************************************************
#define FLASH_USERREG3_DATA_M   0xFFFFFFFF  // User Data
#define FLASH_USERREG3_DATA_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID0 register.
//
//*****************************************************************************
#define SYSCTL_DID0_VER_M       0x70000000  // DID0 Version
#define SYSCTL_DID0_VER_1       0x10000000  // Second version of the DID0
                                            // register format.
#define SYSCTL_DID0_CLASS_M     0x00FF0000  // Device Class
#define SYSCTL_DID0_CLASS_TM4C123                                             \
                                0x00050000  // Tiva TM4C123x and TM4E123x
                                            // microcontrollers
#define SYSCTL_DID0_MAJ_M       0x0000FF00  // Major Revision
#define SYSCTL_DID0_MAJ_REVA    0x00000000  // Revision A (initial device)
#define SYSCTL_DID0_MAJ_REVB    0x00000100  // Revision B (first base layer
                                            // revision)
#define SYSCTL_DID0_MAJ_REVC    0x00000200  // Revision C (second base layer
                                            // revision)
#define SYSCTL_DID0_MIN_M       0x000000FF  // Minor Revision
#define SYSCTL_DID0_MIN_0       0x00000000  // Initial device, or a major
                                            // revision update
#define SYSCTL_DID0_MIN_1       0x00000001  // First metal layer change
#define SYSCTL_DID0_MIN_2       0x00000002  // Second metal layer change

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID1 register.
//
//*****************************************************************************
#define SYSCTL_DID1_VER_M       0xF0000000  // DID1 Version
#define SYSCTL_DID1_VER_1       0x10000000  // fury_ib
#define SYSCTL_DID1_FAM_M       0x0F000000  // Family
#define SYSCTL_DID1_FAM_TIVA    0x00000000  // Tiva family of microcontollers
#define SYSCTL_DID1_PRTNO_M     0x00FF0000  // Part Number
#define SYSCTL_DID1_PRTNO_TM4C123GH6PM                                        \
                                0x00A10000  // TM4C123GH6PM
#define SYSCTL_DID1_PINCNT_M    0x0000E000  // Package Pin Count
#define SYSCTL_DID1_PINCNT_100  0x00004000  // 100-pin LQFP package
#define SYSCTL_DID1_PINCNT_64   0x00006000  // 64-pin LQFP package
#define SYSCTL_DID1_PINCNT_144  0x00008000  // 144-pin LQFP package
#define SYSCTL_DID1_PINCNT_157  0x0000A000  // 157-pin BGA package
#define SYSCTL_DID1_PINCNT_128  0x0000C000  // 128-pin TQFP package
#define SYSCTL_DID1_TEMP_M      0x000000E0  // Temperature Range
#define SYSCTL_DID1_TEMP_I      0x00000020  // Industrial temperature range
#define SYSCTL_DID1_TEMP_E      0x00000040  // Extended temperature range
#define SYSCTL_DID1_TEMP_IE     0x00000060  // Available in both industrial
                                            // temperature range (-40C to 85C)
                                            // and extended temperature range
                                            // (-40C to 105C) devices. See
#define SYSCTL_DID1_PKG_M       0x00000018  // Package Type
#define SYSCTL_DID1_PKG_QFP     0x00000008  // QFP package
#define SYSCTL_DID1_PKG_BGA     0x00000010  // BGA package
#define SYSCTL_DID1_ROHS        0x00000004  // RoHS-Compliance
#define SYSCTL_DID1_QUAL_M      0x00000003  // Qualification Status
#define SYSCTL_DID1_QUAL_ES     0x00000000  // Engineering Sample (unqualified)
#define SYSCTL_DID1_QUAL_PP     0x00000001  // Pilot Production (unqualified)
#define SYSCTL_DID1_QUAL_FQ     0x00000002  // Fully Qualified

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC0 register.
//
//*****************************************************************************
#define SYSCTL_DC0_SRAMSZ_M     0xFFFF0000  // SRAM Size
#define SYSCTL_DC0_SRAMSZ_2KB   0x00070000  // 2 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_4KB   0x000F0000  // 4 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_6KB   0x00170000  // 6 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_8KB   0x001F0000  // 8 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_12KB  0x002F0000  // 12 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_16KB  0x003F0000  // 16 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_20KB  0x004F0000  // 20 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_24KB  0x005F0000  // 24 KB of SRAM
#define SYSCTL_DC0_SRAMSZ_32KB  0x007F0000  // 32 KB of SRAM
#define SYSCTL_DC0_FLASHSZ_M    0x0000FFFF  // Flash Size
#define SYSCTL_DC0_FLASHSZ_8KB  0x00000003  // 8 KB of Flash
#define SYSCTL_DC0_FLASHSZ_16KB 0x00000007  // 16 KB of Flash
#define SYSCTL_DC0_FLASHSZ_32KB 0x0000000F  // 32 KB of Flash
#define SYSCTL_DC0_FLASHSZ_64KB 0x0000001F  // 64 KB of Flash
#define SYSCTL_DC0_FLASHSZ_96KB 0x0000002F  // 96 KB of Flash
#define SYSCTL_DC0_FLASHSZ_128K 0x0000003F  // 128 KB of Flash
#define SYSCTL_DC0_FLASHSZ_192K 0x0000005F  // 192 KB of Flash
#define SYSCTL_DC0_FLASHSZ_256K 0x0000007F  // 256 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC1 register.
//
//*****************************************************************************
#define SYSCTL_DC1_WDT1         0x10000000  // Watchdog Timer1 Present
#define SYSCTL_DC1_CAN1         0x02000000  // CAN Module 1 Present
#define SYSCTL_DC1_CAN0         0x01000000  // CAN Module 0 Present
#define SYSCTL_DC1_PWM1         0x00200000  // PWM Module 1 Present
#define SYSCTL_DC1_PWM0         0x00100000  // PWM Module 0 Present
#define SYSCTL_DC1_ADC1         0x00020000  // ADC Module 1 Present
#define SYSCTL_DC1_ADC0         0x00010000  // ADC Module 0 Present
#define SYSCTL_DC1_MINSYSDIV_M  0x0000F000  // System Clock Divider
#define SYSCTL_DC1_MINSYSDIV_80 0x00002000  // Specifies an 80-MHz CPU clock
                                            // with a PLL divider of 2.5
#define SYSCTL_DC1_MINSYSDIV_50 0x00003000  // Specifies a 50-MHz CPU clock
                                            // with a PLL divider of 4
#define SYSCTL_DC1_MINSYSDIV_40 0x00004000  // Specifies a 40-MHz CPU clock
                                            // with a PLL divider of 5
#define SYSCTL_DC1_MINSYSDIV_25 0x00007000  // Specifies a 25-MHz clock with a
                                            // PLL divider of 8
#define SYSCTL_DC1_MINSYSDIV_20 0x00009000  // Specifies a 20-MHz clock with a
                                            // PLL divider of 10
#define SYSCTL_DC1_ADC1SPD_M    0x00000C00  // Max ADC1 Speed
#define SYSCTL_DC1_ADC1SPD_125K 0x00000000  // 125K samples/second
#define SYSCTL_DC1_ADC1SPD_250K 0x00000400  // 250K samples/second
#define SYSCTL_DC1_ADC1SPD_500K 0x00000800  // 500K samples/second
#define SYSCTL_DC1_ADC1SPD_1M   0x00000C00  // 1M samples/second
#define SYSCTL_DC1_ADC0SPD_M    0x00000300  // Max ADC0 Speed
#define SYSCTL_DC1_ADC0SPD_125K 0x00000000  // 125K samples/second
#define SYSCTL_DC1_ADC0SPD_250K 0x00000100  // 250K samples/second
#define SYSCTL_DC1_ADC0SPD_500K 0x00000200  // 500K samples/second
#define SYSCTL_DC1_ADC0SPD_1M   0x00000300  // 1M samples/second
#define SYSCTL_DC1_MPU          0x00000080  // MPU Present
#define SYSCTL_DC1_HIB          0x00000040  // Hibernation Module Present
#define SYSCTL_DC1_TEMP         0x00000020  // Temp Sensor Present
#define SYSCTL_DC1_PLL          0x00000010  // PLL Present
#define SYSCTL_DC1_WDT0         0x00000008  // Watchdog Timer 0 Present
#define SYSCTL_DC1_SWO          0x00000004  // SWO Trace Port Present
#define SYSCTL_DC1_SWD          0x00000002  // SWD Present
#define SYSCTL_DC1_JTAG         0x00000001  // JTAG Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC2 register.
//
//*****************************************************************************
#define SYSCTL_DC2_EPI0         0x40000000  // EPI Module 0 Present
#define SYSCTL_DC2_I2S0         0x10000000  // I2S Module 0 Present
#define SYSCTL_DC2_COMP2        0x04000000  // Analog Comparator 2 Present
#define SYSCTL_DC2_COMP1        0x02000000  // Analog Comparator 1 Present
#define SYSCTL_DC2_COMP0        0x01000000  // Analog Comparator 0 Present
#define SYSCTL_DC2_TIMER3       0x00080000  // Timer Module 3 Present
#define SYSCTL_DC2_TIMER2       0x00040000  // Timer Module 2 Present
#define SYSCTL_DC2_TIMER1       0x00020000  // Timer Module 1 Present
#define SYSCTL_DC2_TIMER0       0x00010000  // Timer Module 0 Present
#define SYSCTL_DC2_I2C1HS       0x00008000  // I2C Module 1 Speed
#define SYSCTL_DC2_I2C1         0x00004000  // I2C Module 1 Present
#define SYSCTL_DC2_I2C0HS       0x00002000  // I2C Module 0 Speed
#define SYSCTL_DC2_I2C0         0x00001000  // I2C Module 0 Present
#define SYSCTL_DC2_QEI1         0x00000200  // QEI Module 1 Present
#define SYSCTL_DC2_QEI0         0x00000100  // QEI Module 0 Present
#define SYSCTL_DC2_SSI1         0x00000020  // SSI Module 1 Present
#define SYSCTL_DC2_SSI0         0x00000010  // SSI Module 0 Present
#define SYSCTL_DC2_UART2        0x00000004  // UART Module 2 Present
#define SYSCTL_DC2_UART1        0x00000002  // UART Module 1 Present
#define SYSCTL_DC2_UART0        0x00000001  // UART Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC3 register.
//
//*****************************************************************************
#define SYSCTL_DC3_32KHZ        0x80000000  // 32KHz Input Clock Available
#define SYSCTL_DC3_CCP5         0x20000000  // T2CCP1 Pin Present
#define SYSCTL_DC3_CCP4         0x10000000  // T2CCP0 Pin Present
#define SYSCTL_DC3_CCP3         0x08000000  // T1CCP1 Pin Present
#define SYSCTL_DC3_CCP2         0x04000000  // T1CCP0 Pin Present
#define SYSCTL_DC3_CCP1         0x02000000  // T0CCP1 Pin Present
#define SYSCTL_DC3_CCP0         0x01000000  // T0CCP0 Pin Present
#define SYSCTL_DC3_ADC0AIN7     0x00800000  // ADC Module 0 AIN7 Pin Present
#define SYSCTL_DC3_ADC0AIN6     0x00400000  // ADC Module 0 AIN6 Pin Present
#define SYSCTL_DC3_ADC0AIN5     0x00200000  // ADC Module 0 AIN5 Pin Present
#define SYSCTL_DC3_ADC0AIN4     0x00100000  // ADC Module 0 AIN4 Pin Present
#define SYSCTL_DC3_ADC0AIN3     0x00080000  // ADC Module 0 AIN3 Pin Present
#define SYSCTL_DC3_ADC0AIN2     0x00040000  // ADC Module 0 AIN2 Pin Present
#define SYSCTL_DC3_ADC0AIN1     0x00020000  // ADC Module 0 AIN1 Pin Present
#define SYSCTL_DC3_ADC0AIN0     0x00010000  // ADC Module 0 AIN0 Pin Present
#define SYSCTL_DC3_PWMFAULT     0x00008000  // PWM Fault Pin Present
#define SYSCTL_DC3_C2O          0x00004000  // C2o Pin Present
#define SYSCTL_DC3_C2PLUS       0x00002000  // C2+ Pin Present
#define SYSCTL_DC3_C2MINUS      0x00001000  // C2- Pin Present
#define SYSCTL_DC3_C1O          0x00000800  // C1o Pin Present
#define SYSCTL_DC3_C1PLUS       0x00000400  // C1+ Pin Present
#define SYSCTL_DC3_C1MINUS      0x00000200  // C1- Pin Present
#define SYSCTL_DC3_C0O          0x00000100  // C0o Pin Present
#define SYSCTL_DC3_C0PLUS       0x00000080  // C0+ Pin Present
#define SYSCTL_DC3_C0MINUS      0x00000040  // C0- Pin Present
#define SYSCTL_DC3_PWM5         0x00000020  // PWM5 Pin Present
#define SYSCTL_DC3_PWM4         0x00000010  // PWM4 Pin Present
#define SYSCTL_DC3_PWM3         0x00000008  // PWM3 Pin Present
#define SYSCTL_DC3_PWM2         0x00000004  // PWM2 Pin Present
#define SYSCTL_DC3_PWM1         0x00000002  // PWM1 Pin Present
#define SYSCTL_DC3_PWM0         0x00000001  // PWM0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC4 register.
//
//*****************************************************************************
#define SYSCTL_DC4_EPHY0        0x40000000  // Ethernet PHY Layer 0 Present
#define SYSCTL_DC4_EMAC0        0x10000000  // Ethernet MAC Layer 0 Present
#define SYSCTL_DC4_E1588        0x01000000  // 1588 Capable
#define SYSCTL_DC4_PICAL        0x00040000  // PIOSC Calibrate
#define SYSCTL_DC4_CCP7         0x00008000  // T3CCP1 Pin Present
#define SYSCTL_DC4_CCP6         0x00004000  // T3CCP0 Pin Present
#define SYSCTL_DC4_UDMA         0x00002000  // Micro-DMA Module Present
#define SYSCTL_DC4_ROM          0x00001000  // Internal Code ROM Present
#define SYSCTL_DC4_GPIOJ        0x00000100  // GPIO Port J Present
#define SYSCTL_DC4_GPIOH        0x00000080  // GPIO Port H Present
#define SYSCTL_DC4_GPIOG        0x00000040  // GPIO Port G Present
#define SYSCTL_DC4_GPIOF        0x00000020  // GPIO Port F Present
#define SYSCTL_DC4_GPIOE        0x00000010  // GPIO Port E Present
#define SYSCTL_DC4_GPIOD        0x00000008  // GPIO Port D Present
#define SYSCTL_DC4_GPIOC        0x00000004  // GPIO Port C Present
#define SYSCTL_DC4_GPIOB        0x00000002  // GPIO Port B Present
#define SYSCTL_DC4_GPIOA        0x00000001  // GPIO Port A Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC5 register.
//
//*****************************************************************************
#define SYSCTL_DC5_PWMFAULT3    0x08000000  // PWM Fault 3 Pin Present
#define SYSCTL_DC5_PWMFAULT2    0x04000000  // PWM Fault 2 Pin Present
#define SYSCTL_DC5_PWMFAULT1    0x02000000  // PWM Fault 1 Pin Present
#define SYSCTL_DC5_PWMFAULT0    0x01000000  // PWM Fault 0 Pin Present
#define SYSCTL_DC5_PWMEFLT      0x00200000  // PWM Extended Fault Active
#define SYSCTL_DC5_PWMESYNC     0x00100000  // PWM Extended SYNC Active
#define SYSCTL_DC5_PWM7         0x00000080  // PWM7 Pin Present
#define SYSCTL_DC5_PWM6         0x00000040  // PWM6 Pin Present
#define SYSCTL_DC5_PWM5         0x00000020  // PWM5 Pin Present
#define SYSCTL_DC5_PWM4         0x00000010  // PWM4 Pin Present
#define SYSCTL_DC5_PWM3         0x00000008  // PWM3 Pin Present
#define SYSCTL_DC5_PWM2         0x00000004  // PWM2 Pin Present
#define SYSCTL_DC5_PWM1         0x00000002  // PWM1 Pin Present
#define SYSCTL_DC5_PWM0         0x00000001  // PWM0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC6 register.
//
//*****************************************************************************
#define SYSCTL_DC6_USB0PHY      0x00000010  // USB Module 0 PHY Present
#define SYSCTL_DC6_USB0_M       0x00000003  // USB Module 0 Present
#define SYSCTL_DC6_USB0_DEV     0x00000001  // USB0 is Device Only
#define SYSCTL_DC6_USB0_HOSTDEV 0x00000002  // USB is Device or Host
#define SYSCTL_DC6_USB0_OTG     0x00000003  // USB0 is OTG

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC7 register.
//
//*****************************************************************************
#define SYSCTL_DC7_DMACH30      0x40000000  // DMA Channel 30
#define SYSCTL_DC7_DMACH29      0x20000000  // DMA Channel 29
#define SYSCTL_DC7_DMACH28      0x10000000  // DMA Channel 28
#define SYSCTL_DC7_DMACH27      0x08000000  // DMA Channel 27
#define SYSCTL_DC7_DMACH26      0x04000000  // DMA Channel 26
#define SYSCTL_DC7_DMACH25      0x02000000  // DMA Channel 25
#define SYSCTL_DC7_DMACH24      0x01000000  // DMA Channel 24
#define SYSCTL_DC7_DMACH23      0x00800000  // DMA Channel 23
#define SYSCTL_DC7_DMACH22      0x00400000  // DMA Channel 22
#define SYSCTL_DC7_DMACH21      0x00200000  // DMA Channel 21
#define SYSCTL_DC7_DMACH20      0x00100000  // DMA Channel 20
#define SYSCTL_DC7_DMACH19      0x00080000  // DMA Channel 19
#define SYSCTL_DC7_DMACH18      0x00040000  // DMA Channel 18
#define SYSCTL_DC7_DMACH17      0x00020000  // DMA Channel 17
#define SYSCTL_DC7_DMACH16      0x00010000  // DMA Channel 16
#define SYSCTL_DC7_DMACH15      0x00008000  // DMA Channel 15
#define SYSCTL_DC7_DMACH14      0x00004000  // DMA Channel 14
#define SYSCTL_DC7_DMACH13      0x00002000  // DMA Channel 13
#define SYSCTL_DC7_DMACH12      0x00001000  // DMA Channel 12
#define SYSCTL_DC7_DMACH11      0x00000800  // DMA Channel 11
#define SYSCTL_DC7_DMACH10      0x00000400  // DMA Channel 10
#define SYSCTL_DC7_DMACH9       0x00000200  // DMA Channel 9
#define SYSCTL_DC7_DMACH8       0x00000100  // DMA Channel 8
#define SYSCTL_DC7_DMACH7       0x00000080  // DMA Channel 7
#define SYSCTL_DC7_DMACH6       0x00000040  // DMA Channel 6
#define SYSCTL_DC7_DMACH5       0x00000020  // DMA Channel 5
#define SYSCTL_DC7_DMACH4       0x00000010  // DMA Channel 4
#define SYSCTL_DC7_DMACH3       0x00000008  // DMA Channel 3
#define SYSCTL_DC7_DMACH2       0x00000004  // DMA Channel 2
#define SYSCTL_DC7_DMACH1       0x00000002  // DMA Channel 1
#define SYSCTL_DC7_DMACH0       0x00000001  // DMA Channel 0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC8 register.
//
//*****************************************************************************
#define SYSCTL_DC8_ADC1AIN15    0x80000000  // ADC Module 1 AIN15 Pin Present
#define SYSCTL_DC8_ADC1AIN14    0x40000000  // ADC Module 1 AIN14 Pin Present
#define SYSCTL_DC8_ADC1AIN13    0x20000000  // ADC Module 1 AIN13 Pin Present
#define SYSCTL_DC8_ADC1AIN12    0x10000000  // ADC Module 1 AIN12 Pin Present
#define SYSCTL_DC8_ADC1AIN11    0x08000000  // ADC Module 1 AIN11 Pin Present
#define SYSCTL_DC8_ADC1AIN10    0x04000000  // ADC Module 1 AIN10 Pin Present
#define SYSCTL_DC8_ADC1AIN9     0x02000000  // ADC Module 1 AIN9 Pin Present
#define SYSCTL_DC8_ADC1AIN8     0x01000000  // ADC Module 1 AIN8 Pin Present
#define SYSCTL_DC8_ADC1AIN7     0x00800000  // ADC Module 1 AIN7 Pin Present
#define SYSCTL_DC8_ADC1AIN6     0x00400000  // ADC Module 1 AIN6 Pin Present
#define SYSCTL_DC8_ADC1AIN5     0x00200000  // ADC Module 1 AIN5 Pin Present
#define SYSCTL_DC8_ADC1AIN4     0x00100000  // ADC Module 1 AIN4 Pin Present
#define SYSCTL_DC8_ADC1AIN3     0x00080000  // ADC Module 1 AIN3 Pin Present
#define SYSCTL_DC8_ADC1AIN2     0x00040000  // ADC Module 1 AIN2 Pin Present
#define SYSCTL_DC8_ADC1AIN1     0x00020000  // ADC Module 1 AIN1 Pin Present
#define SYSCTL_DC8_ADC1AIN0     0x00010000  // ADC Module 1 AIN0 Pin Present
#define SYSCTL_DC8_ADC0AIN15    0x00008000  // ADC Module 0 AIN15 Pin Present
#define SYSCTL_DC8_ADC0AIN14    0x00004000  // ADC Module 0 AIN14 Pin Present
#define SYSCTL_DC8_ADC0AIN13    0x00002000  // ADC Module 0 AIN13 Pin Present
#define SYSCTL_DC8_ADC0AIN12    0x00001000  // ADC Module 0 AIN12 Pin Present
#define SYSCTL_DC8_ADC0AIN11    0x00000800  // ADC Module 0 AIN11 Pin Present
#define SYSCTL_DC8_ADC0AIN10    0x00000400  // ADC Module 0 AIN10 Pin Present
#define SYSCTL_DC8_ADC0AIN9     0x00000200  // ADC Module 0 AIN9 Pin Present
#define SYSCTL_DC8_ADC0AIN8     0x00000100  // ADC Module 0 AIN8 Pin Present
#define SYSCTL_DC8_ADC0AIN7     0x00000080  // ADC Module 0 AIN7 Pin Present
#define SYSCTL_DC8_ADC0AIN6     0x00000040  // ADC Module 0 AIN6 Pin Present
#define SYSCTL_DC8_ADC0AIN5     0x00000020  // ADC Module 0 AIN5 Pin Present
#define SYSCTL_DC8_ADC0AIN4     0x00000010  // ADC Module 0 AIN4 Pin Present
#define SYSCTL_DC8_ADC0AIN3     0x00000008  // ADC Module 0 AIN3 Pin Present
#define SYSCTL_DC8_ADC0AIN2     0x00000004  // ADC Module 0 AIN2 Pin Present
#define SYSCTL_DC8_ADC0AIN1     0x00000002  // ADC Module 0 AIN1 Pin Present
#define SYSCTL_DC8_ADC0AIN0     0x00000001  // ADC Module 0 AIN0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PBORCTL register.
//
//*****************************************************************************
#define SYSCTL_PBORCTL_BOR0     0x00000004  // VDD under BOR0 Event Action
#define SYSCTL_PBORCTL_BOR1     0x00000002  // VDD under BOR1 Event Action

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR0 register.
//
//*****************************************************************************
#define SYSCTL_SRCR0_WDT1       0x10000000  // WDT1 Reset Control
#define SYSCTL_SRCR0_CAN1       0x02000000  // CAN1 Reset Control
#define SYSCTL_SRCR0_CAN0       0x01000000  // CAN0 Reset Control
#define SYSCTL_SRCR0_PWM0       0x00100000  // PWM Reset Control
#define SYSCTL_SRCR0_ADC1       0x00020000  // ADC1 Reset Control
#define SYSCTL_SRCR0_ADC0       0x00010000  // ADC0 Reset Control
#define SYSCTL_SRCR0_HIB        0x00000040  // HIB Reset Control
#define SYSCTL_SRCR0_WDT0       0x00000008  // WDT0 Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR1 register.
//
//*****************************************************************************
#define SYSCTL_SRCR1_COMP1      0x02000000  // Analog Comp 1 Reset Control
#define SYSCTL_SRCR1_COMP0      0x01000000  // Analog Comp 0 Reset Control
#define SYSCTL_SRCR1_TIMER3     0x00080000  // Timer 3 Reset Control
#define SYSCTL_SRCR1_TIMER2     0x00040000  // Timer 2 Reset Control
#define SYSCTL_SRCR1_TIMER1     0x00020000  // Timer 1 Reset Control
#define SYSCTL_SRCR1_TIMER0     0x00010000  // Timer 0 Reset Control
#define SYSCTL_SRCR1_I2C1       0x00004000  // I2C1 Reset Control
#define SYSCTL_SRCR1_I2C0       0x00001000  // I2C0 Reset Control
#define SYSCTL_SRCR1_QEI1       0x00000200  // QEI1 Reset Control
#define SYSCTL_SRCR1_QEI0       0x00000100  // QEI0 Reset Control
#define SYSCTL_SRCR1_SSI1       0x00000020  // SSI1 Reset Control
#define SYSCTL_SRCR1_SSI0       0x00000010  // SSI0 Reset Control
#define SYSCTL_SRCR1_UART2      0x00000004  // UART2 Reset Control
#define SYSCTL_SRCR1_UART1      0x00000002  // UART1 Reset Control
#define SYSCTL_SRCR1_UART0      0x00000001  // UART0 Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR2 register.
//
//*****************************************************************************
#define SYSCTL_SRCR2_USB0       0x00010000  // USB0 Reset Control
#define SYSCTL_SRCR2_UDMA       0x00002000  // Micro-DMA Reset Control
#define SYSCTL_SRCR2_GPIOF      0x00000020  // Port F Reset Control
#define SYSCTL_SRCR2_GPIOE      0x00000010  // Port E Reset Control
#define SYSCTL_SRCR2_GPIOD      0x00000008  // Port D Reset Control
#define SYSCTL_SRCR2_GPIOC      0x00000004  // Port C Reset Control
#define SYSCTL_SRCR2_GPIOB      0x00000002  // Port B Reset Control
#define SYSCTL_SRCR2_GPIOA      0x00000001  // Port A Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RIS register.
//
//*****************************************************************************
#define SYSCTL_RIS_BOR0RIS      0x00000800  // VDD under BOR0 Raw Interrupt
                                            // Status
#define SYSCTL_RIS_VDDARIS      0x00000400  // VDDA Power OK Event Raw
                                            // Interrupt Status
#define SYSCTL_RIS_MOSCPUPRIS   0x00000100  // MOSC Power Up Raw Interrupt
                                            // Status
#define SYSCTL_RIS_USBPLLLRIS   0x00000080  // USB PLL Lock Raw Interrupt
                                            // Status
#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status
#define SYSCTL_RIS_MOFRIS       0x00000008  // Main Oscillator Failure Raw
                                            // Interrupt Status
#define SYSCTL_RIS_BOR1RIS      0x00000002  // VDD under BOR1 Raw Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_IMC register.
//
//*****************************************************************************
#define SYSCTL_IMC_BOR0IM       0x00000800  // VDD under BOR0 Interrupt Mask
#define SYSCTL_IMC_VDDAIM       0x00000400  // VDDA Power OK Interrupt Mask
#define SYSCTL_IMC_MOSCPUPIM    0x00000100  // MOSC Power Up Interrupt Mask
#define SYSCTL_IMC_USBPLLLIM    0x00000080  // USB PLL Lock Interrupt Mask
#define SYSCTL_IMC_PLLLIM       0x00000040  // PLL Lock Interrupt Mask
#define SYSCTL_IMC_MOFIM        0x00000008  // Main Oscillator Failure
                                            // Interrupt Mask
#define SYSCTL_IMC_BOR1IM       0x00000002  // VDD under BOR1 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MISC register.
//
//*****************************************************************************
#define SYSCTL_MISC_BOR0MIS     0x00000800  // VDD under BOR0 Masked Interrupt
                                            // Status
#define SYSCTL_MISC_VDDAMIS     0x00000400  // VDDA Power OK Masked Interrupt
                                            // Status
#define SYSCTL_MISC_MOSCPUPMIS  0x00000100  // MOSC Power Up Masked Interrupt
                                            // Status
#define SYSCTL_MISC_USBPLLLMIS  0x00000080  // USB PLL Lock Masked Interrupt
                                            // Status
#define SYSCTL_MISC_PLLLMIS     0x00000040  // PLL Lock Masked Interrupt Status
#define SYSCTL_MISC_MOFMIS      0x00000008  // Main Oscillator Failure Masked
                                            // Interrupt Status
#define SYSCTL_MISC_BOR1MIS     0x00000002  // VDD under BOR1 Masked Interrupt
                                            // Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESC register.
//
//*****************************************************************************
#define SYSCTL_RESC_MOSCFAIL    0x00010000  // MOSC Failure Reset
#define SYSCTL_RESC_WDT1        0x00000020  // Watchdog Timer 1 Reset
#define SYSCTL_RESC_SW          0x00000010  // Software Reset
#define SYSCTL_RESC_WDT0        0x00000008  // Watchdog Timer 0 Reset
#define SYSCTL_RESC_BOR         0x00000004  // Brown-Out Reset
#define SYSCTL_RESC_POR         0x00000002  // Power-On Reset
#define SYSCTL_RESC_EXT         0x00000001  // External Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC register.
//
//*****************************************************************************
#define SYSCTL_RCC_ACG          0x08000000  // Auto Clock Gating
#define SYSCTL_RCC_SYSDIV_M     0x07800000  // System Clock Divisor
#define SYSCTL_RCC_USESYSDIV    0x00400000  // Enable System Clock Divider
#define SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2     0x00000000  // PWM clock /2
#define SYSCTL_RCC_PWMDIV_4     0x00020000  // PWM clock /4
#define SYSCTL_RCC_PWMDIV_8     0x00040000  // PWM clock /8
#define SYSCTL_RCC_PWMDIV_16    0x00060000  // PWM clock /16
#define SYSCTL_RCC_PWMDIV_32    0x00080000  // PWM clock /32
#define SYSCTL_RCC_PWMDIV_64    0x000A0000  // PWM clock /64
#define SYSCTL_RCC_PWRDN        0x00002000  // PLL Power Down
#define SYSCTL_RCC_BYPASS       0x00000800  // PLL Bypass
#define SYSCTL_RCC_XTAL_M       0x000007C0  // Crystal Value
#define SYSCTL_RCC_XTAL_4MHZ    0x00000180  // 4 MHz
#define SYSCTL_RCC_XTAL_4_09MHZ 0x000001C0  // 4.096 MHz
#define SYSCTL_RCC_XTAL_4_91MHZ 0x00000200  // 4.9152 MHz
#define SYSCTL_RCC_XTAL_5MHZ    0x00000240  // 5 MHz
#define SYSCTL_RCC_XTAL_5_12MHZ 0x00000280  // 5.12 MHz
#define SYSCTL_RCC_XTAL_6MHZ    0x000002C0  // 6 MHz
#define SYSCTL_RCC_XTAL_6_14MHZ 0x00000300  // 6.144 MHz
#define SYSCTL_RCC_XTAL_7_37MHZ 0x00000340  // 7.3728 MHz
#define SYSCTL_RCC_XTAL_8MHZ    0x00000380  // 8 MHz
#define SYSCTL_RCC_XTAL_8_19MHZ 0x000003C0  // 8.192 MHz
#define SYSCTL_RCC_XTAL_10MHZ   0x00000400  // 10 MHz
#define SYSCTL_RCC_XTAL_12MHZ   0x00000440  // 12 MHz
#define SYSCTL_RCC_XTAL_12_2MHZ 0x00000480  // 12.288 MHz
#define SYSCTL_RCC_XTAL_13_5MHZ 0x000004C0  // 13.56 MHz
#define SYSCTL_RCC_XTAL_14_3MHZ 0x00000500  // 14.31818 MHz
#define SYSCTL_RCC_XTAL_16MHZ   0x00000540  // 16 MHz
#define SYSCTL_RCC_XTAL_16_3MHZ 0x00000580  // 16.384 MHz
#define SYSCTL_RCC_XTAL_18MHZ   0x000005C0  // 18.0 MHz (USB)
#define SYSCTL_RCC_XTAL_20MHZ   0x00000600  // 20.0 MHz (USB)
#define SYSCTL_RCC_XTAL_24MHZ   0x00000640  // 24.0 MHz (USB)
#define SYSCTL_RCC_XTAL_25MHZ   0x00000680  // 25.0 MHz (USB)
#define SYSCTL_RCC_OSCSRC_M     0x00000030  // Oscillator Source
#define SYSCTL_RCC_OSCSRC_MAIN  0x00000000  // MOSC
#define SYSCTL_RCC_OSCSRC_INT   0x00000010  // IOSC
#define SYSCTL_RCC_OSCSRC_INT4  0x00000020  // IOSC/4
#define SYSCTL_RCC_OSCSRC_30    0x00000030  // LFIOSC
#define SYSCTL_RCC_MOSCDIS      0x00000001  // Main Oscillator Disable
#define SYSCTL_RCC_SYSDIV_S     23

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_GPIOHBCTL
// register.
//
//*****************************************************************************
#define SYSCTL_GPIOHBCTL_PORTF  0x00000020  // Port F Advanced High-Performance
                                            // Bus
#define SYSCTL_GPIOHBCTL_PORTE  0x00000010  // Port E Advanced High-Performance
                                            // Bus
#define SYSCTL_GPIOHBCTL_PORTD  0x00000008  // Port D Advanced High-Performance
                                            // Bus
#define SYSCTL_GPIOHBCTL_PORTC  0x00000004  // Port C Advanced High-Performance
                                            // Bus
#define SYSCTL_GPIOHBCTL_PORTB  0x00000002  // Port B Advanced High-Performance
                                            // Bus
#define SYSCTL_GPIOHBCTL_PORTA  0x00000001  // Port A Advanced High-Performance
                                            // Bus

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC2 register.
//
//*****************************************************************************
#define SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define SYSCTL_RCC2_DIV400      0x40000000  // Divide PLL as 400 MHz vs. 200
                                            // MHz
#define SYSCTL_RCC2_SYSDIV2_M   0x1F800000  // System Clock Divisor 2
#define SYSCTL_RCC2_SYSDIV2LSB  0x00400000  // Additional LSB for SYSDIV2
#define SYSCTL_RCC2_USBPWRDN    0x00004000  // Power-Down USB PLL
#define SYSCTL_RCC2_PWRDN2      0x00002000  // Power-Down PLL 2
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2
#define SYSCTL_RCC2_OSCSRC2_M   0x00000070  // Oscillator Source 2
#define SYSCTL_RCC2_OSCSRC2_MO  0x00000000  // MOSC
#define SYSCTL_RCC2_OSCSRC2_IO  0x00000010  // PIOSC
#define SYSCTL_RCC2_OSCSRC2_IO4 0x00000020  // PIOSC/4
#define SYSCTL_RCC2_OSCSRC2_30  0x00000030  // LFIOSC
#define SYSCTL_RCC2_OSCSRC2_32  0x00000070  // 32.768 kHz
#define SYSCTL_RCC2_SYSDIV2_S   23

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MOSCCTL register.
//
//*****************************************************************************
#define SYSCTL_MOSCCTL_NOXTAL   0x00000004  // No Crystal Connected
#define SYSCTL_MOSCCTL_MOSCIM   0x00000002  // MOSC Failure Action
#define SYSCTL_MOSCCTL_CVAL     0x00000001  // Clock Validation for MOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC0 register.
//
//*****************************************************************************
#define SYSCTL_RCGC0_WDT1       0x10000000  // WDT1 Clock Gating Control
#define SYSCTL_RCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control
#define SYSCTL_RCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control
#define SYSCTL_RCGC0_PWM0       0x00100000  // PWM Clock Gating Control
#define SYSCTL_RCGC0_ADC1       0x00020000  // ADC1 Clock Gating Control
#define SYSCTL_RCGC0_ADC0       0x00010000  // ADC0 Clock Gating Control
#define SYSCTL_RCGC0_ADC1SPD_M  0x00000C00  // ADC1 Sample Speed
#define SYSCTL_RCGC0_ADC1SPD_125K                                             \
                                0x00000000  // 125K samples/second
#define SYSCTL_RCGC0_ADC1SPD_250K                                             \
                                0x00000400  // 250K samples/second
#define SYSCTL_RCGC0_ADC1SPD_500K                                             \
                                0x00000800  // 500K samples/second
#define SYSCTL_RCGC0_ADC1SPD_1M 0x00000C00  // 1M samples/second
#define SYSCTL_RCGC0_ADC0SPD_M  0x00000300  // ADC0 Sample Speed
#define SYSCTL_RCGC0_ADC0SPD_125K                                             \
                                0x00000000  // 125K samples/second
#define SYSCTL_RCGC0_ADC0SPD_250K                                             \
                                0x00000100  // 250K samples/second
#define SYSCTL_RCGC0_ADC0SPD_500K                                             \
                                0x00000200  // 500K samples/second
#define SYSCTL_RCGC0_ADC0SPD_1M 0x00000300  // 1M samples/second
#define SYSCTL_RCGC0_HIB        0x00000040  // HIB Clock Gating Control
#define SYSCTL_RCGC0_WDT0       0x00000008  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC1 register.
//
//*****************************************************************************
#define SYSCTL_RCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock Gating
#define SYSCTL_RCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock Gating
#define SYSCTL_RCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control
#define SYSCTL_RCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control
#define SYSCTL_RCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control
#define SYSCTL_RCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control
#define SYSCTL_RCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control
#define SYSCTL_RCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control
#define SYSCTL_RCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control
#define SYSCTL_RCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control
#define SYSCTL_RCGC1_SSI1       0x00000020  // SSI1 Clock Gating Control
#define SYSCTL_RCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define SYSCTL_RCGC1_UART2      0x00000004  // UART2 Clock Gating Control
#define SYSCTL_RCGC1_UART1      0x00000002  // UART1 Clock Gating Control
#define SYSCTL_RCGC1_UART0      0x00000001  // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC2 register.
//
//*****************************************************************************
#define SYSCTL_RCGC2_USB0       0x00010000  // USB0 Clock Gating Control
#define SYSCTL_RCGC2_UDMA       0x00002000  // Micro-DMA Clock Gating Control
#define SYSCTL_RCGC2_GPIOF      0x00000020  // Port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOD      0x00000008  // Port D Clock Gating Control
#define SYSCTL_RCGC2_GPIOC      0x00000004  // Port C Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC0 register.
//
//*****************************************************************************
#define SYSCTL_SCGC0_WDT1       0x10000000  // WDT1 Clock Gating Control
#define SYSCTL_SCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control
#define SYSCTL_SCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control
#define SYSCTL_SCGC0_PWM0       0x00100000  // PWM Clock Gating Control
#define SYSCTL_SCGC0_ADC1       0x00020000  // ADC1 Clock Gating Control
#define SYSCTL_SCGC0_ADC0       0x00010000  // ADC0 Clock Gating Control
#define SYSCTL_SCGC0_HIB        0x00000040  // HIB Clock Gating Control
#define SYSCTL_SCGC0_WDT0       0x00000008  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC1 register.
//
//*****************************************************************************
#define SYSCTL_SCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock Gating
#define SYSCTL_SCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock Gating
#define SYSCTL_SCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control
#define SYSCTL_SCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control
#define SYSCTL_SCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control
#define SYSCTL_SCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control
#define SYSCTL_SCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control
#define SYSCTL_SCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control
#define SYSCTL_SCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control
#define SYSCTL_SCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control
#define SYSCTL_SCGC1_SSI1       0x00000020  // SSI1 Clock Gating Control
#define SYSCTL_SCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define SYSCTL_SCGC1_UART2      0x00000004  // UART2 Clock Gating Control
#define SYSCTL_SCGC1_UART1      0x00000002  // UART1 Clock Gating Control
#define SYSCTL_SCGC1_UART0      0x00000001  // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC2 register.
//
//*****************************************************************************
#define SYSCTL_SCGC2_USB0       0x00010000  // USB0 Clock Gating Control
#define SYSCTL_SCGC2_UDMA       0x00002000  // Micro-DMA Clock Gating Control
#define SYSCTL_SCGC2_GPIOF      0x00000020  // Port F Clock Gating Control
#define SYSCTL_SCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define SYSCTL_SCGC2_GPIOD      0x00000008  // Port D Clock Gating Control
#define SYSCTL_SCGC2_GPIOC      0x00000004  // Port C Clock Gating Control
#define SYSCTL_SCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define SYSCTL_SCGC2_GPIOA      0x00000001  // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC0 register.
//
//*****************************************************************************
#define SYSCTL_DCGC0_WDT1       0x10000000  // WDT1 Clock Gating Control
#define SYSCTL_DCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control
#define SYSCTL_DCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control
#define SYSCTL_DCGC0_PWM0       0x00100000  // PWM Clock Gating Control
#define SYSCTL_DCGC0_ADC1       0x00020000  // ADC1 Clock Gating Control
#define SYSCTL_DCGC0_ADC0       0x00010000  // ADC0 Clock Gating Control
#define SYSCTL_DCGC0_HIB        0x00000040  // HIB Clock Gating Control
#define SYSCTL_DCGC0_WDT0       0x00000008  // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC1 register.
//
//*****************************************************************************
#define SYSCTL_DCGC1_COMP1      0x02000000  // Analog Comparator 1 Clock Gating
#define SYSCTL_DCGC1_COMP0      0x01000000  // Analog Comparator 0 Clock Gating
#define SYSCTL_DCGC1_TIMER3     0x00080000  // Timer 3 Clock Gating Control
#define SYSCTL_DCGC1_TIMER2     0x00040000  // Timer 2 Clock Gating Control
#define SYSCTL_DCGC1_TIMER1     0x00020000  // Timer 1 Clock Gating Control
#define SYSCTL_DCGC1_TIMER0     0x00010000  // Timer 0 Clock Gating Control
#define SYSCTL_DCGC1_I2C1       0x00004000  // I2C1 Clock Gating Control
#define SYSCTL_DCGC1_I2C0       0x00001000  // I2C0 Clock Gating Control
#define SYSCTL_DCGC1_QEI1       0x00000200  // QEI1 Clock Gating Control
#define SYSCTL_DCGC1_QEI0       0x00000100  // QEI0 Clock Gating Control
#define SYSCTL_DCGC1_SSI1       0x00000020  // SSI1 Clock Gating Control
#define SYSCTL_DCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define SYSCTL_DCGC1_UART2      0x00000004  // UART2 Clock Gating Control
#define SYSCTL_DCGC1_UART1      0x00000002  // UART1 Clock Gating Control
#define SYSCTL_DCGC1_UART0      0x00000001  // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC2 register.
//
//*****************************************************************************
#define SYSCTL_DCGC2_USB0       0x00010000  // USB0 Clock Gating Control
#define SYSCTL_DCGC2_UDMA       0x00002000  // Micro-DMA Clock Gating Control
#define SYSCTL_DCGC2_GPIOF      0x00000020  // Port F Clock Gating Control
#define SYSCTL_DCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define SYSCTL_DCGC2_GPIOD      0x00000008  // Port D Clock Gating Control
#define SYSCTL_DCGC2_GPIOC      0x00000004  // Port C Clock Gating Control
#define SYSCTL_DCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define SYSCTL_DCGC2_GPIOA      0x00000001  // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPCLKCFG
// register.
//
//*****************************************************************************
#define SYSCTL_DSLPCLKCFG_D_M   0x1F800000  // Divider Field Override
#define SYSCTL_DSLPCLKCFG_O_M   0x00000070  // Clock Source
#define SYSCTL_DSLPCLKCFG_O_IGN 0x00000000  // MOSC
#define SYSCTL_DSLPCLKCFG_O_IO  0x00000010  // PIOSC
#define SYSCTL_DSLPCLKCFG_O_30  0x00000030  // LFIOSC
#define SYSCTL_DSLPCLKCFG_O_32  0x00000070  // 32.768 kHz
#define SYSCTL_DSLPCLKCFG_PIOSCPD                                             \
                                0x00000002  // PIOSC Power Down Request
#define SYSCTL_DSLPCLKCFG_D_S   23

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SYSPROP register.
//
//*****************************************************************************
#define SYSCTL_SYSPROP_FPU      0x00000001  // FPU Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCCAL
// register.
//
//*****************************************************************************
#define SYSCTL_PIOSCCAL_UTEN    0x80000000  // Use User Trim Value
#define SYSCTL_PIOSCCAL_CAL     0x00000200  // Start Calibration
#define SYSCTL_PIOSCCAL_UPDATE  0x00000100  // Update Trim
#define SYSCTL_PIOSCCAL_UT_M    0x0000007F  // User Trim Value
#define SYSCTL_PIOSCCAL_UT_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCSTAT
// register.
//
//*****************************************************************************
#define SYSCTL_PIOSCSTAT_DT_M   0x007F0000  // Default Trim Value
#define SYSCTL_PIOSCSTAT_CR_M   0x00000300  // Calibration Result
#define SYSCTL_PIOSCSTAT_CRNONE 0x00000000  // Calibration has not been
                                            // attempted
#define SYSCTL_PIOSCSTAT_CRPASS 0x00000100  // The last calibration operation
                                            // completed to meet 1% accuracy
#define SYSCTL_PIOSCSTAT_CRFAIL 0x00000200  // The last calibration operation
                                            // failed to meet 1% accuracy
#define SYSCTL_PIOSCSTAT_CT_M   0x0000007F  // Calibration Trim Value
#define SYSCTL_PIOSCSTAT_DT_S   16
#define SYSCTL_PIOSCSTAT_CT_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ0
// register.
//
//*****************************************************************************
#define SYSCTL_PLLFREQ0_MFRAC_M 0x000FFC00  // PLL M Fractional Value
#define SYSCTL_PLLFREQ0_MINT_M  0x000003FF  // PLL M Integer Value
#define SYSCTL_PLLFREQ0_MFRAC_S 10
#define SYSCTL_PLLFREQ0_MINT_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ1
// register.
//
//*****************************************************************************
#define SYSCTL_PLLFREQ1_Q_M     0x00001F00  // PLL Q Value
#define SYSCTL_PLLFREQ1_N_M     0x0000001F  // PLL N Value
#define SYSCTL_PLLFREQ1_Q_S     8
#define SYSCTL_PLLFREQ1_N_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLSTAT register.
//
//*****************************************************************************
#define SYSCTL_PLLSTAT_LOCK     0x00000001  // PLL Lock

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SLPPWRCFG
// register.
//
//*****************************************************************************
#define SYSCTL_SLPPWRCFG_FLASHPM_M                                            \
                                0x00000030  // Flash Power Modes
#define SYSCTL_SLPPWRCFG_FLASHPM_NRM                                          \
                                0x00000000  // Active Mode
#define SYSCTL_SLPPWRCFG_FLASHPM_SLP                                          \
                                0x00000020  // Low Power Mode
#define SYSCTL_SLPPWRCFG_SRAMPM_M                                             \
                                0x00000003  // SRAM Power Modes
#define SYSCTL_SLPPWRCFG_SRAMPM_NRM                                           \
                                0x00000000  // Active Mode
#define SYSCTL_SLPPWRCFG_SRAMPM_SBY                                           \
                                0x00000001  // Standby Mode
#define SYSCTL_SLPPWRCFG_SRAMPM_LP                                            \
                                0x00000003  // Low Power Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPPWRCFG
// register.
//
//*****************************************************************************
#define SYSCTL_DSLPPWRCFG_FLASHPM_M                                           \
                                0x00000030  // Flash Power Modes
#define SYSCTL_DSLPPWRCFG_FLASHPM_NRM                                         \
                                0x00000000  // Active Mode
#define SYSCTL_DSLPPWRCFG_FLASHPM_SLP                                         \
                                0x00000020  // Low Power Mode
#define SYSCTL_DSLPPWRCFG_SRAMPM_M                                            \
                                0x00000003  // SRAM Power Modes
#define SYSCTL_DSLPPWRCFG_SRAMPM_NRM                                          \
                                0x00000000  // Active Mode
#define SYSCTL_DSLPPWRCFG_SRAMPM_SBY                                          \
                                0x00000001  // Standby Mode
#define SYSCTL_DSLPPWRCFG_SRAMPM_LP                                           \
                                0x00000003  // Low Power Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC9 register.
//
//*****************************************************************************
#define SYSCTL_DC9_ADC1DC7      0x00800000  // ADC1 DC7 Present
#define SYSCTL_DC9_ADC1DC6      0x00400000  // ADC1 DC6 Present
#define SYSCTL_DC9_ADC1DC5      0x00200000  // ADC1 DC5 Present
#define SYSCTL_DC9_ADC1DC4      0x00100000  // ADC1 DC4 Present
#define SYSCTL_DC9_ADC1DC3      0x00080000  // ADC1 DC3 Present
#define SYSCTL_DC9_ADC1DC2      0x00040000  // ADC1 DC2 Present
#define SYSCTL_DC9_ADC1DC1      0x00020000  // ADC1 DC1 Present
#define SYSCTL_DC9_ADC1DC0      0x00010000  // ADC1 DC0 Present
#define SYSCTL_DC9_ADC0DC7      0x00000080  // ADC0 DC7 Present
#define SYSCTL_DC9_ADC0DC6      0x00000040  // ADC0 DC6 Present
#define SYSCTL_DC9_ADC0DC5      0x00000020  // ADC0 DC5 Present
#define SYSCTL_DC9_ADC0DC4      0x00000010  // ADC0 DC4 Present
#define SYSCTL_DC9_ADC0DC3      0x00000008  // ADC0 DC3 Present
#define SYSCTL_DC9_ADC0DC2      0x00000004  // ADC0 DC2 Present
#define SYSCTL_DC9_ADC0DC1      0x00000002  // ADC0 DC1 Present
#define SYSCTL_DC9_ADC0DC0      0x00000001  // ADC0 DC0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_NVMSTAT register.
//
//*****************************************************************************
#define SYSCTL_NVMSTAT_FWB      0x00000001  // 32 Word Flash Write Buffer
                                            // Available

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDOSPCTL
// register.
//
//*****************************************************************************
#define SYSCTL_LDOSPCTL_VADJEN  0x80000000  // Voltage Adjust Enable
#define SYSCTL_LDOSPCTL_VLDO_M  0x000000FF  // LDO Output Voltage
#define SYSCTL_LDOSPCTL_VLDO_0_90V                                            \
                                0x00000012  // 0.90 V
#define SYSCTL_LDOSPCTL_VLDO_0_95V                                            \
                                0x00000013  // 0.95 V
#define SYSCTL_LDOSPCTL_VLDO_1_00V                                            \
                                0x00000014  // 1.00 V
#define SYSCTL_LDOSPCTL_VLDO_1_05V                                            \
                                0x00000015  // 1.05 V
#define SYSCTL_LDOSPCTL_VLDO_1_10V                                            \
                                0x00000016  // 1.10 V
#define SYSCTL_LDOSPCTL_VLDO_1_15V                                            \
                                0x00000017  // 1.15 V
#define SYSCTL_LDOSPCTL_VLDO_1_20V                                            \
                                0x00000018  // 1.20 V

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDODPCTL
// register.
//
//*****************************************************************************
#define SYSCTL_LDODPCTL_VADJEN  0x80000000  // Voltage Adjust Enable
#define SYSCTL_LDODPCTL_VLDO_M  0x000000FF  // LDO Output Voltage
#define SYSCTL_LDODPCTL_VLDO_0_90V                                            \
                                0x00000012  // 0.90 V
#define SYSCTL_LDODPCTL_VLDO_0_95V                                            \
                                0x00000013  // 0.95 V
#define SYSCTL_LDODPCTL_VLDO_1_00V                                            \
                                0x00000014  // 1.00 V
#define SYSCTL_LDODPCTL_VLDO_1_05V                                            \
                                0x00000015  // 1.05 V
#define SYSCTL_LDODPCTL_VLDO_1_10V                                            \
                                0x00000016  // 1.10 V
#define SYSCTL_LDODPCTL_VLDO_1_15V                                            \
                                0x00000017  // 1.15 V
#define SYSCTL_LDODPCTL_VLDO_1_20V                                            \
                                0x00000018  // 1.20 V

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWD register.
//
//*****************************************************************************
#define SYSCTL_PPWD_P1          0x00000002  // Watchdog Timer 1 Present
#define SYSCTL_PPWD_P0          0x00000001  // Watchdog Timer 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPTIMER register.
//
//*****************************************************************************
#define SYSCTL_PPTIMER_P5       0x00000020  // 16/32-Bit General-Purpose Timer
                                            // 5 Present
#define SYSCTL_PPTIMER_P4       0x00000010  // 16/32-Bit General-Purpose Timer
                                            // 4 Present
#define SYSCTL_PPTIMER_P3       0x00000008  // 16/32-Bit General-Purpose Timer
                                            // 3 Present
#define SYSCTL_PPTIMER_P2       0x00000004  // 16/32-Bit General-Purpose Timer
                                            // 2 Present
#define SYSCTL_PPTIMER_P1       0x00000002  // 16/32-Bit General-Purpose Timer
                                            // 1 Present
#define SYSCTL_PPTIMER_P0       0x00000001  // 16/32-Bit General-Purpose Timer
                                            // 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPGPIO register.
//
//*****************************************************************************
#define SYSCTL_PPGPIO_P14       0x00004000  // GPIO Port Q Present
#define SYSCTL_PPGPIO_P13       0x00002000  // GPIO Port P Present
#define SYSCTL_PPGPIO_P12       0x00001000  // GPIO Port N Present
#define SYSCTL_PPGPIO_P11       0x00000800  // GPIO Port M Present
#define SYSCTL_PPGPIO_P10       0x00000400  // GPIO Port L Present
#define SYSCTL_PPGPIO_P9        0x00000200  // GPIO Port K Present
#define SYSCTL_PPGPIO_P8        0x00000100  // GPIO Port J Present
#define SYSCTL_PPGPIO_P7        0x00000080  // GPIO Port H Present
#define SYSCTL_PPGPIO_P6        0x00000040  // GPIO Port G Present
#define SYSCTL_PPGPIO_P5        0x00000020  // GPIO Port F Present
#define SYSCTL_PPGPIO_P4        0x00000010  // GPIO Port E Present
#define SYSCTL_PPGPIO_P3        0x00000008  // GPIO Port D Present
#define SYSCTL_PPGPIO_P2        0x00000004  // GPIO Port C Present
#define SYSCTL_PPGPIO_P1        0x00000002  // GPIO Port B Present
#define SYSCTL_PPGPIO_P0        0x00000001  // GPIO Port A Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPDMA register.
//
//*****************************************************************************
#define SYSCTL_PPDMA_P0         0x00000001  // uDMA Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPHIB register.
//
//*****************************************************************************
#define SYSCTL_PPHIB_P0         0x00000001  // Hibernation Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUART register.
//
//*****************************************************************************
#define SYSCTL_PPUART_P7        0x00000080  // UART Module 7 Present
#define SYSCTL_PPUART_P6        0x00000040  // UART Module 6 Present
#define SYSCTL_PPUART_P5        0x00000020  // UART Module 5 Present
#define SYSCTL_PPUART_P4        0x00000010  // UART Module 4 Present
#define SYSCTL_PPUART_P3        0x00000008  // UART Module 3 Present
#define SYSCTL_PPUART_P2        0x00000004  // UART Module 2 Present
#define SYSCTL_PPUART_P1        0x00000002  // UART Module 1 Present
#define SYSCTL_PPUART_P0        0x00000001  // UART Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPSSI register.
//
//*****************************************************************************
#define SYSCTL_PPSSI_P3         0x00000008  // SSI Module 3 Present
#define SYSCTL_PPSSI_P2         0x00000004  // SSI Module 2 Present
#define SYSCTL_PPSSI_P1         0x00000002  // SSI Module 1 Present
#define SYSCTL_PPSSI_P0         0x00000001  // SSI Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPI2C register.
//
//*****************************************************************************
#define SYSCTL_PPI2C_P5         0x00000020  // I2C Module 5 Present
#define SYSCTL_PPI2C_P4         0x00000010  // I2C Module 4 Present
#define SYSCTL_PPI2C_P3         0x00000008  // I2C Module 3 Present
#define SYSCTL_PPI2C_P2         0x00000004  // I2C Module 2 Present
#define SYSCTL_PPI2C_P1         0x00000002  // I2C Module 1 Present
#define SYSCTL_PPI2C_P0         0x00000001  // I2C Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUSB register.
//
//*****************************************************************************
#define SYSCTL_PPUSB_P0         0x00000001  // USB Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPCAN register.
//
//*****************************************************************************
#define SYSCTL_PPCAN_P1         0x00000002  // CAN Module 1 Present
#define SYSCTL_PPCAN_P0         0x00000001  // CAN Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPADC register.
//
//*****************************************************************************
#define SYSCTL_PPADC_P1         0x00000002  // ADC Module 1 Present
#define SYSCTL_PPADC_P0         0x00000001  // ADC Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPACMP register.
//
//*****************************************************************************
#define SYSCTL_PPACMP_P0        0x00000001  // Analog Comparator Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPPWM register.
//
//*****************************************************************************
#define SYSCTL_PPPWM_P1         0x00000002  // PWM Module 1 Present
#define SYSCTL_PPPWM_P0         0x00000001  // PWM Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPQEI register.
//
//*****************************************************************************
#define SYSCTL_PPQEI_P1         0x00000002  // QEI Module 1 Present
#define SYSCTL_PPQEI_P0         0x00000001  // QEI Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEEPROM
// register.
//
//*****************************************************************************
#define SYSCTL_PPEEPROM_P0      0x00000001  // EEPROM Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_PPWTIMER_P5      0x00000020  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Present
#define SYSCTL_PPWTIMER_P4      0x00000010  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Present
#define SYSCTL_PPWTIMER_P3      0x00000008  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Present
#define SYSCTL_PPWTIMER_P2      0x00000004  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Present
#define SYSCTL_PPWTIMER_P1      0x00000002  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Present
#define SYSCTL_PPWTIMER_P0      0x00000001  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWD register.
//
//*****************************************************************************
#define SYSCTL_SRWD_R1          0x00000002  // Watchdog Timer 1 Software Reset
#define SYSCTL_SRWD_R0          0x00000001  // Watchdog Timer 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRTIMER register.
//
//*****************************************************************************
#define SYSCTL_SRTIMER_R5       0x00000020  // 16/32-Bit General-Purpose Timer
                                            // 5 Software Reset
#define SYSCTL_SRTIMER_R4       0x00000010  // 16/32-Bit General-Purpose Timer
                                            // 4 Software Reset
#define SYSCTL_SRTIMER_R3       0x00000008  // 16/32-Bit General-Purpose Timer
                                            // 3 Software Reset
#define SYSCTL_SRTIMER_R2       0x00000004  // 16/32-Bit General-Purpose Timer
                                            // 2 Software Reset
#define SYSCTL_SRTIMER_R1       0x00000002  // 16/32-Bit General-Purpose Timer
                                            // 1 Software Reset
#define SYSCTL_SRTIMER_R0       0x00000001  // 16/32-Bit General-Purpose Timer
                                            // 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRGPIO register.
//
//*****************************************************************************
#define SYSCTL_SRGPIO_R5        0x00000020  // GPIO Port F Software Reset
#define SYSCTL_SRGPIO_R4        0x00000010  // GPIO Port E Software Reset
#define SYSCTL_SRGPIO_R3        0x00000008  // GPIO Port D Software Reset
#define SYSCTL_SRGPIO_R2        0x00000004  // GPIO Port C Software Reset
#define SYSCTL_SRGPIO_R1        0x00000002  // GPIO Port B Software Reset
#define SYSCTL_SRGPIO_R0        0x00000001  // GPIO Port A Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRDMA register.
//
//*****************************************************************************
#define SYSCTL_SRDMA_R0         0x00000001  // uDMA Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRHIB register.
//
//*****************************************************************************
#define SYSCTL_SRHIB_R0         0x00000001  // Hibernation Module Software
                                            // Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUART register.
//
//*****************************************************************************
#define SYSCTL_SRUART_R7        0x00000080  // UART Module 7 Software Reset
#define SYSCTL_SRUART_R6        0x00000040  // UART Module 6 Software Reset
#define SYSCTL_SRUART_R5        0x00000020  // UART Module 5 Software Reset
#define SYSCTL_SRUART_R4        0x00000010  // UART Module 4 Software Reset
#define SYSCTL_SRUART_R3        0x00000008  // UART Module 3 Software Reset
#define SYSCTL_SRUART_R2        0x00000004  // UART Module 2 Software Reset
#define SYSCTL_SRUART_R1        0x00000002  // UART Module 1 Software Reset
#define SYSCTL_SRUART_R0        0x00000001  // UART Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRSSI register.
//
//*****************************************************************************
#define SYSCTL_SRSSI_R3         0x00000008  // SSI Module 3 Software Reset
#define SYSCTL_SRSSI_R2         0x00000004  // SSI Module 2 Software Reset
#define SYSCTL_SRSSI_R1         0x00000002  // SSI Module 1 Software Reset
#define SYSCTL_SRSSI_R0         0x00000001  // SSI Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRI2C register.
//
//*****************************************************************************
#define SYSCTL_SRI2C_R3         0x00000008  // I2C Module 3 Software Reset
#define SYSCTL_SRI2C_R2         0x00000004  // I2C Module 2 Software Reset
#define SYSCTL_SRI2C_R1         0x00000002  // I2C Module 1 Software Reset
#define SYSCTL_SRI2C_R0         0x00000001  // I2C Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUSB register.
//
//*****************************************************************************
#define SYSCTL_SRUSB_R0         0x00000001  // USB Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCAN register.
//
//*****************************************************************************
#define SYSCTL_SRCAN_R1         0x00000002  // CAN Module 1 Software Reset
#define SYSCTL_SRCAN_R0         0x00000001  // CAN Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRADC register.
//
//*****************************************************************************
#define SYSCTL_SRADC_R1         0x00000002  // ADC Module 1 Software Reset
#define SYSCTL_SRADC_R0         0x00000001  // ADC Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRACMP register.
//
//*****************************************************************************
#define SYSCTL_SRACMP_R0        0x00000001  // Analog Comparator Module 0
                                            // Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRPWM register.
//
//*****************************************************************************
#define SYSCTL_SRPWM_R1         0x00000002  // PWM Module 1 Software Reset
#define SYSCTL_SRPWM_R0         0x00000001  // PWM Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRQEI register.
//
//*****************************************************************************
#define SYSCTL_SRQEI_R1         0x00000002  // QEI Module 1 Software Reset
#define SYSCTL_SRQEI_R0         0x00000001  // QEI Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREEPROM
// register.
//
//*****************************************************************************
#define SYSCTL_SREEPROM_R0      0x00000001  // EEPROM Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_SRWTIMER_R5      0x00000020  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Software Reset
#define SYSCTL_SRWTIMER_R4      0x00000010  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Software Reset
#define SYSCTL_SRWTIMER_R3      0x00000008  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Software Reset
#define SYSCTL_SRWTIMER_R2      0x00000004  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Software Reset
#define SYSCTL_SRWTIMER_R1      0x00000002  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Software Reset
#define SYSCTL_SRWTIMER_R0      0x00000001  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWD register.
//
//*****************************************************************************
#define SYSCTL_RCGCWD_R1        0x00000002  // Watchdog Timer 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCWD_R0        0x00000001  // Watchdog Timer 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_RCGCTIMER_R5     0x00000020  // 16/32-Bit General-Purpose Timer
                                            // 5 Run Mode Clock Gating Control
#define SYSCTL_RCGCTIMER_R4     0x00000010  // 16/32-Bit General-Purpose Timer
                                            // 4 Run Mode Clock Gating Control
#define SYSCTL_RCGCTIMER_R3     0x00000008  // 16/32-Bit General-Purpose Timer
                                            // 3 Run Mode Clock Gating Control
#define SYSCTL_RCGCTIMER_R2     0x00000004  // 16/32-Bit General-Purpose Timer
                                            // 2 Run Mode Clock Gating Control
#define SYSCTL_RCGCTIMER_R1     0x00000002  // 16/32-Bit General-Purpose Timer
                                            // 1 Run Mode Clock Gating Control
#define SYSCTL_RCGCTIMER_R0     0x00000001  // 16/32-Bit General-Purpose Timer
                                            // 0 Run Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCGPIO
// register.
//
//*****************************************************************************
#define SYSCTL_RCGCGPIO_R5      0x00000020  // GPIO Port F Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R4      0x00000010  // GPIO Port E Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R3      0x00000008  // GPIO Port D Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R2      0x00000004  // GPIO Port C Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R1      0x00000002  // GPIO Port B Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R0      0x00000001  // GPIO Port A Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCDMA register.
//
//*****************************************************************************
#define SYSCTL_RCGCDMA_R0       0x00000001  // uDMA Module Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCHIB register.
//
//*****************************************************************************
#define SYSCTL_RCGCHIB_R0       0x00000001  // Hibernation Module Run Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUART
// register.
//
//*****************************************************************************
#define SYSCTL_RCGCUART_R7      0x00000080  // UART Module 7 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCUART_R6      0x00000040  // UART Module 6 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCUART_R5      0x00000020  // UART Module 5 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCUART_R4      0x00000010  // UART Module 4 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCUART_R3      0x00000008  // UART Module 3 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCUART_R2      0x00000004  // UART Module 2 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCUART_R1      0x00000002  // UART Module 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCUART_R0      0x00000001  // UART Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCSSI register.
//
//*****************************************************************************
#define SYSCTL_RCGCSSI_R3       0x00000008  // SSI Module 3 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCSSI_R2       0x00000004  // SSI Module 2 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCSSI_R1       0x00000002  // SSI Module 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCSSI_R0       0x00000001  // SSI Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCI2C register.
//
//*****************************************************************************
#define SYSCTL_RCGCI2C_R3       0x00000008  // I2C Module 3 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCI2C_R2       0x00000004  // I2C Module 2 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCI2C_R1       0x00000002  // I2C Module 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCI2C_R0       0x00000001  // I2C Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUSB register.
//
//*****************************************************************************
#define SYSCTL_RCGCUSB_R0       0x00000001  // USB Module Run Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCCAN register.
//
//*****************************************************************************
#define SYSCTL_RCGCCAN_R1       0x00000002  // CAN Module 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCCAN_R0       0x00000001  // CAN Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCADC register.
//
//*****************************************************************************
#define SYSCTL_RCGCADC_R1       0x00000002  // ADC Module 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCADC_R0       0x00000001  // ADC Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCACMP
// register.
//
//*****************************************************************************
#define SYSCTL_RCGCACMP_R0      0x00000001  // Analog Comparator Module 0 Run
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCPWM register.
//
//*****************************************************************************
#define SYSCTL_RCGCPWM_R1       0x00000002  // PWM Module 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCPWM_R0       0x00000001  // PWM Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCQEI register.
//
//*****************************************************************************
#define SYSCTL_RCGCQEI_R1       0x00000002  // QEI Module 1 Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCQEI_R0       0x00000001  // QEI Module 0 Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEEPROM
// register.
//
//*****************************************************************************
#define SYSCTL_RCGCEEPROM_R0    0x00000001  // EEPROM Module Run Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_RCGCWTIMER_R5    0x00000020  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Run Mode Clock Gating
                                            // Control
#define SYSCTL_RCGCWTIMER_R4    0x00000010  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Run Mode Clock Gating
                                            // Control
#define SYSCTL_RCGCWTIMER_R3    0x00000008  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Run Mode Clock Gating
                                            // Control
#define SYSCTL_RCGCWTIMER_R2    0x00000004  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Run Mode Clock Gating
                                            // Control
#define SYSCTL_RCGCWTIMER_R1    0x00000002  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Run Mode Clock Gating
                                            // Control
#define SYSCTL_RCGCWTIMER_R0    0x00000001  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Run Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWD register.
//
//*****************************************************************************
#define SYSCTL_SCGCWD_S1        0x00000002  // Watchdog Timer 1 Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_SCGCWD_S0        0x00000001  // Watchdog Timer 0 Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_SCGCTIMER_S5     0x00000020  // 16/32-Bit General-Purpose Timer
                                            // 5 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCTIMER_S4     0x00000010  // 16/32-Bit General-Purpose Timer
                                            // 4 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCTIMER_S3     0x00000008  // 16/32-Bit General-Purpose Timer
                                            // 3 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCTIMER_S2     0x00000004  // 16/32-Bit General-Purpose Timer
                                            // 2 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCTIMER_S1     0x00000002  // 16/32-Bit General-Purpose Timer
                                            // 1 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCTIMER_S0     0x00000001  // 16/32-Bit General-Purpose Timer
                                            // 0 Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCGPIO
// register.
//
//*****************************************************************************
#define SYSCTL_SCGCGPIO_S5      0x00000020  // GPIO Port F Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCGPIO_S4      0x00000010  // GPIO Port E Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCGPIO_S3      0x00000008  // GPIO Port D Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCGPIO_S2      0x00000004  // GPIO Port C Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCGPIO_S1      0x00000002  // GPIO Port B Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCGPIO_S0      0x00000001  // GPIO Port A Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCDMA register.
//
//*****************************************************************************
#define SYSCTL_SCGCDMA_S0       0x00000001  // uDMA Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCHIB register.
//
//*****************************************************************************
#define SYSCTL_SCGCHIB_S0       0x00000001  // Hibernation Module Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUART
// register.
//
//*****************************************************************************
#define SYSCTL_SCGCUART_S7      0x00000080  // UART Module 7 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCUART_S6      0x00000040  // UART Module 6 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCUART_S5      0x00000020  // UART Module 5 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCUART_S4      0x00000010  // UART Module 4 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCUART_S3      0x00000008  // UART Module 3 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCUART_S2      0x00000004  // UART Module 2 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCUART_S1      0x00000002  // UART Module 1 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCUART_S0      0x00000001  // UART Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCSSI register.
//
//*****************************************************************************
#define SYSCTL_SCGCSSI_S3       0x00000008  // SSI Module 3 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCSSI_S2       0x00000004  // SSI Module 2 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCSSI_S1       0x00000002  // SSI Module 1 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCSSI_S0       0x00000001  // SSI Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCI2C register.
//
//*****************************************************************************
#define SYSCTL_SCGCI2C_S3       0x00000008  // I2C Module 3 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCI2C_S2       0x00000004  // I2C Module 2 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCI2C_S1       0x00000002  // I2C Module 1 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCI2C_S0       0x00000001  // I2C Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUSB register.
//
//*****************************************************************************
#define SYSCTL_SCGCUSB_S0       0x00000001  // USB Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCCAN register.
//
//*****************************************************************************
#define SYSCTL_SCGCCAN_S1       0x00000002  // CAN Module 1 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCCAN_S0       0x00000001  // CAN Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCADC register.
//
//*****************************************************************************
#define SYSCTL_SCGCADC_S1       0x00000002  // ADC Module 1 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCADC_S0       0x00000001  // ADC Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCACMP
// register.
//
//*****************************************************************************
#define SYSCTL_SCGCACMP_S0      0x00000001  // Analog Comparator Module 0 Sleep
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCPWM register.
//
//*****************************************************************************
#define SYSCTL_SCGCPWM_S1       0x00000002  // PWM Module 1 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCPWM_S0       0x00000001  // PWM Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCQEI register.
//
//*****************************************************************************
#define SYSCTL_SCGCQEI_S1       0x00000002  // QEI Module 1 Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_SCGCQEI_S0       0x00000001  // QEI Module 0 Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEEPROM
// register.
//
//*****************************************************************************
#define SYSCTL_SCGCEEPROM_S0    0x00000001  // EEPROM Module Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_SCGCWTIMER_S5    0x00000020  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCWTIMER_S4    0x00000010  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCWTIMER_S3    0x00000008  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCWTIMER_S2    0x00000004  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCWTIMER_S1    0x00000002  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_SCGCWTIMER_S0    0x00000001  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWD register.
//
//*****************************************************************************
#define SYSCTL_DCGCWD_D1        0x00000002  // Watchdog Timer 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCWD_D0        0x00000001  // Watchdog Timer 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_DCGCTIMER_D5     0x00000020  // 16/32-Bit General-Purpose Timer
                                            // 5 Deep-Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_DCGCTIMER_D4     0x00000010  // 16/32-Bit General-Purpose Timer
                                            // 4 Deep-Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_DCGCTIMER_D3     0x00000008  // 16/32-Bit General-Purpose Timer
                                            // 3 Deep-Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_DCGCTIMER_D2     0x00000004  // 16/32-Bit General-Purpose Timer
                                            // 2 Deep-Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_DCGCTIMER_D1     0x00000002  // 16/32-Bit General-Purpose Timer
                                            // 1 Deep-Sleep Mode Clock Gating
                                            // Control
#define SYSCTL_DCGCTIMER_D0     0x00000001  // 16/32-Bit General-Purpose Timer
                                            // 0 Deep-Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCGPIO
// register.
//
//*****************************************************************************
#define SYSCTL_DCGCGPIO_D5      0x00000020  // GPIO Port F Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCGPIO_D4      0x00000010  // GPIO Port E Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCGPIO_D3      0x00000008  // GPIO Port D Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCGPIO_D2      0x00000004  // GPIO Port C Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCGPIO_D1      0x00000002  // GPIO Port B Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCGPIO_D0      0x00000001  // GPIO Port A Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCDMA register.
//
//*****************************************************************************
#define SYSCTL_DCGCDMA_D0       0x00000001  // uDMA Module Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCHIB register.
//
//*****************************************************************************
#define SYSCTL_DCGCHIB_D0       0x00000001  // Hibernation Module Deep-Sleep
                                            // Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUART
// register.
//
//*****************************************************************************
#define SYSCTL_DCGCUART_D7      0x00000080  // UART Module 7 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCUART_D6      0x00000040  // UART Module 6 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCUART_D5      0x00000020  // UART Module 5 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCUART_D4      0x00000010  // UART Module 4 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCUART_D3      0x00000008  // UART Module 3 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCUART_D2      0x00000004  // UART Module 2 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCUART_D1      0x00000002  // UART Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCUART_D0      0x00000001  // UART Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCSSI register.
//
//*****************************************************************************
#define SYSCTL_DCGCSSI_D3       0x00000008  // SSI Module 3 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCSSI_D2       0x00000004  // SSI Module 2 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCSSI_D1       0x00000002  // SSI Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCSSI_D0       0x00000001  // SSI Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCI2C register.
//
//*****************************************************************************
#define SYSCTL_DCGCI2C_D3       0x00000008  // I2C Module 3 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCI2C_D2       0x00000004  // I2C Module 2 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCI2C_D1       0x00000002  // I2C Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCI2C_D0       0x00000001  // I2C Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUSB register.
//
//*****************************************************************************
#define SYSCTL_DCGCUSB_D0       0x00000001  // USB Module Deep-Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCCAN register.
//
//*****************************************************************************
#define SYSCTL_DCGCCAN_D1       0x00000002  // CAN Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCCAN_D0       0x00000001  // CAN Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCADC register.
//
//*****************************************************************************
#define SYSCTL_DCGCADC_D1       0x00000002  // ADC Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCADC_D0       0x00000001  // ADC Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCACMP
// register.
//
//*****************************************************************************
#define SYSCTL_DCGCACMP_D0      0x00000001  // Analog Comparator Module 0
                                            // Deep-Sleep Mode Clock Gating
                                            // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCPWM register.
//
//*****************************************************************************
#define SYSCTL_DCGCPWM_D1       0x00000002  // PWM Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCPWM_D0       0x00000001  // PWM Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCQEI register.
//
//*****************************************************************************
#define SYSCTL_DCGCQEI_D1       0x00000002  // QEI Module 1 Deep-Sleep Mode
                                            // Clock Gating Control
#define SYSCTL_DCGCQEI_D0       0x00000001  // QEI Module 0 Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEEPROM
// register.
//
//*****************************************************************************
#define SYSCTL_DCGCEEPROM_D0    0x00000001  // EEPROM Module Deep-Sleep Mode
                                            // Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_DCGCWTIMER_D5    0x00000020  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Deep-Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_DCGCWTIMER_D4    0x00000010  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Deep-Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_DCGCWTIMER_D3    0x00000008  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Deep-Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_DCGCWTIMER_D2    0x00000004  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Deep-Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_DCGCWTIMER_D1    0x00000002  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Deep-Sleep Mode Clock
                                            // Gating Control
#define SYSCTL_DCGCWTIMER_D0    0x00000001  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Deep-Sleep Mode Clock
                                            // Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWD register.
//
//*****************************************************************************
#define SYSCTL_PRWD_R1          0x00000002  // Watchdog Timer 1 Peripheral
                                            // Ready
#define SYSCTL_PRWD_R0          0x00000001  // Watchdog Timer 0 Peripheral
                                            // Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRTIMER register.
//
//*****************************************************************************
#define SYSCTL_PRTIMER_R5       0x00000020  // 16/32-Bit General-Purpose Timer
                                            // 5 Peripheral Ready
#define SYSCTL_PRTIMER_R4       0x00000010  // 16/32-Bit General-Purpose Timer
                                            // 4 Peripheral Ready
#define SYSCTL_PRTIMER_R3       0x00000008  // 16/32-Bit General-Purpose Timer
                                            // 3 Peripheral Ready
#define SYSCTL_PRTIMER_R2       0x00000004  // 16/32-Bit General-Purpose Timer
                                            // 2 Peripheral Ready
#define SYSCTL_PRTIMER_R1       0x00000002  // 16/32-Bit General-Purpose Timer
                                            // 1 Peripheral Ready
#define SYSCTL_PRTIMER_R0       0x00000001  // 16/32-Bit General-Purpose Timer
                                            // 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRGPIO register.
//
//*****************************************************************************
#define SYSCTL_PRGPIO_R5        0x00000020  // GPIO Port F Peripheral Ready
#define SYSCTL_PRGPIO_R4        0x00000010  // GPIO Port E Peripheral Ready
#define SYSCTL_PRGPIO_R3        0x00000008  // GPIO Port D Peripheral Ready
#define SYSCTL_PRGPIO_R2        0x00000004  // GPIO Port C Peripheral Ready
#define SYSCTL_PRGPIO_R1        0x00000002  // GPIO Port B Peripheral Ready
#define SYSCTL_PRGPIO_R0        0x00000001  // GPIO Port A Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRDMA register.
//
//*****************************************************************************
#define SYSCTL_PRDMA_R0         0x00000001  // uDMA Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRHIB register.
//
//*****************************************************************************
#define SYSCTL_PRHIB_R0         0x00000001  // Hibernation Module Peripheral
                                            // Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUART register.
//
//*****************************************************************************
#define SYSCTL_PRUART_R7        0x00000080  // UART Module 7 Peripheral Ready
#define SYSCTL_PRUART_R6        0x00000040  // UART Module 6 Peripheral Ready
#define SYSCTL_PRUART_R5        0x00000020  // UART Module 5 Peripheral Ready
#define SYSCTL_PRUART_R4        0x00000010  // UART Module 4 Peripheral Ready
#define SYSCTL_PRUART_R3        0x00000008  // UART Module 3 Peripheral Ready
#define SYSCTL_PRUART_R2        0x00000004  // UART Module 2 Peripheral Ready
#define SYSCTL_PRUART_R1        0x00000002  // UART Module 1 Peripheral Ready
#define SYSCTL_PRUART_R0        0x00000001  // UART Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRSSI register.
//
//*****************************************************************************
#define SYSCTL_PRSSI_R3         0x00000008  // SSI Module 3 Peripheral Ready
#define SYSCTL_PRSSI_R2         0x00000004  // SSI Module 2 Peripheral Ready
#define SYSCTL_PRSSI_R1         0x00000002  // SSI Module 1 Peripheral Ready
#define SYSCTL_PRSSI_R0         0x00000001  // SSI Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRI2C register.
//
//*****************************************************************************
#define SYSCTL_PRI2C_R3         0x00000008  // I2C Module 3 Peripheral Ready
#define SYSCTL_PRI2C_R2         0x00000004  // I2C Module 2 Peripheral Ready
#define SYSCTL_PRI2C_R1         0x00000002  // I2C Module 1 Peripheral Ready
#define SYSCTL_PRI2C_R0         0x00000001  // I2C Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUSB register.
//
//*****************************************************************************
#define SYSCTL_PRUSB_R0         0x00000001  // USB Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRCAN register.
//
//*****************************************************************************
#define SYSCTL_PRCAN_R1         0x00000002  // CAN Module 1 Peripheral Ready
#define SYSCTL_PRCAN_R0         0x00000001  // CAN Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRADC register.
//
//*****************************************************************************
#define SYSCTL_PRADC_R1         0x00000002  // ADC Module 1 Peripheral Ready
#define SYSCTL_PRADC_R0         0x00000001  // ADC Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRACMP register.
//
//*****************************************************************************
#define SYSCTL_PRACMP_R0        0x00000001  // Analog Comparator Module 0
                                            // Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRPWM register.
//
//*****************************************************************************
#define SYSCTL_PRPWM_R1         0x00000002  // PWM Module 1 Peripheral Ready
#define SYSCTL_PRPWM_R0         0x00000001  // PWM Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRQEI register.
//
//*****************************************************************************
#define SYSCTL_PRQEI_R1         0x00000002  // QEI Module 1 Peripheral Ready
#define SYSCTL_PRQEI_R0         0x00000001  // QEI Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREEPROM
// register.
//
//*****************************************************************************
#define SYSCTL_PREEPROM_R0      0x00000001  // EEPROM Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWTIMER
// register.
//
//*****************************************************************************
#define SYSCTL_PRWTIMER_R5      0x00000020  // 32/64-Bit Wide General-Purpose
                                            // Timer 5 Peripheral Ready
#define SYSCTL_PRWTIMER_R4      0x00000010  // 32/64-Bit Wide General-Purpose
                                            // Timer 4 Peripheral Ready
#define SYSCTL_PRWTIMER_R3      0x00000008  // 32/64-Bit Wide General-Purpose
                                            // Timer 3 Peripheral Ready
#define SYSCTL_PRWTIMER_R2      0x00000004  // 32/64-Bit Wide General-Purpose
                                            // Timer 2 Peripheral Ready
#define SYSCTL_PRWTIMER_R1      0x00000002  // 32/64-Bit Wide General-Purpose
                                            // Timer 1 Peripheral Ready
#define SYSCTL_PRWTIMER_R0      0x00000001  // 32/64-Bit Wide General-Purpose
                                            // Timer 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_STAT register.
//
//*****************************************************************************
#define UDMA_STAT_DMACHANS_M    0x001F0000  // Available uDMA Channels Minus 1
#define UDMA_STAT_STATE_M       0x000000F0  // Control State Machine Status
#define UDMA_STAT_STATE_IDLE    0x00000000  // Idle
#define UDMA_STAT_STATE_RD_CTRL 0x00000010  // Reading channel controller data
#define UDMA_STAT_STATE_RD_SRCENDP                                            \
                                0x00000020  // Reading source end pointer
#define UDMA_STAT_STATE_RD_DSTENDP                                            \
                                0x00000030  // Reading destination end pointer
#define UDMA_STAT_STATE_RD_SRCDAT                                             \
                                0x00000040  // Reading source data
#define UDMA_STAT_STATE_WR_DSTDAT                                             \
                                0x00000050  // Writing destination data
#define UDMA_STAT_STATE_WAIT    0x00000060  // Waiting for uDMA request to
                                            // clear
#define UDMA_STAT_STATE_WR_CTRL 0x00000070  // Writing channel controller data
#define UDMA_STAT_STATE_STALL   0x00000080  // Stalled
#define UDMA_STAT_STATE_DONE    0x00000090  // Done
#define UDMA_STAT_STATE_UNDEF   0x000000A0  // Undefined
#define UDMA_STAT_MASTEN        0x00000001  // Master Enable Status
#define UDMA_STAT_DMACHANS_S    16

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CFG register.
//
//*****************************************************************************
#define UDMA_CFG_MASTEN         0x00000001  // Controller Master Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CTLBASE register.
//
//*****************************************************************************
#define UDMA_CTLBASE_ADDR_M     0xFFFFFC00  // Channel Control Base Address
#define UDMA_CTLBASE_ADDR_S     10

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTBASE register.
//
//*****************************************************************************
#define UDMA_ALTBASE_ADDR_M     0xFFFFFFFF  // Alternate Channel Address
                                            // Pointer
#define UDMA_ALTBASE_ADDR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_WAITSTAT register.
//
//*****************************************************************************
#define UDMA_WAITSTAT_WAITREQ_M 0xFFFFFFFF  // Channel [n] Wait Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_SWREQ register.
//
//*****************************************************************************
#define UDMA_SWREQ_M            0xFFFFFFFF  // Channel [n] Software Request

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTSET
// register.
//
//*****************************************************************************
#define UDMA_USEBURSTSET_SET_M  0xFFFFFFFF  // Channel [n] Useburst Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTCLR
// register.
//
//*****************************************************************************
#define UDMA_USEBURSTCLR_CLR_M  0xFFFFFFFF  // Channel [n] Useburst Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKSET
// register.
//
//*****************************************************************************
#define UDMA_REQMASKSET_SET_M   0xFFFFFFFF  // Channel [n] Request Mask Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKCLR
// register.
//
//*****************************************************************************
#define UDMA_REQMASKCLR_CLR_M   0xFFFFFFFF  // Channel [n] Request Mask Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENASET register.
//
//*****************************************************************************
#define UDMA_ENASET_SET_M       0xFFFFFFFF  // Channel [n] Enable Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENACLR register.
//
//*****************************************************************************
#define UDMA_ENACLR_CLR_M       0xFFFFFFFF  // Clear Channel [n] Enable Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTSET register.
//
//*****************************************************************************
#define UDMA_ALTSET_SET_M       0xFFFFFFFF  // Channel [n] Alternate Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTCLR register.
//
//*****************************************************************************
#define UDMA_ALTCLR_CLR_M       0xFFFFFFFF  // Channel [n] Alternate Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOSET register.
//
//*****************************************************************************
#define UDMA_PRIOSET_SET_M      0xFFFFFFFF  // Channel [n] Priority Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOCLR register.
//
//*****************************************************************************
#define UDMA_PRIOCLR_CLR_M      0xFFFFFFFF  // Channel [n] Priority Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ERRCLR register.
//
//*****************************************************************************
#define UDMA_ERRCLR_ERRCLR      0x00000001  // uDMA Bus Error Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHASGN register.
//
//*****************************************************************************
#define UDMA_CHASGN_M           0xFFFFFFFF  // Channel [n] Assignment Select
#define UDMA_CHASGN_PRIMARY     0x00000000  // Use the primary channel
                                            // assignment
#define UDMA_CHASGN_SECONDARY   0x00000001  // Use the secondary channel
                                            // assignment

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHIS register.
//
//*****************************************************************************
#define UDMA_CHIS_M             0xFFFFFFFF  // Channel [n] Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP0 register.
//
//*****************************************************************************
#define UDMA_CHMAP0_CH7SEL_M    0xF0000000  // uDMA Channel 7 Source Select
#define UDMA_CHMAP0_CH6SEL_M    0x0F000000  // uDMA Channel 6 Source Select
#define UDMA_CHMAP0_CH5SEL_M    0x00F00000  // uDMA Channel 5 Source Select
#define UDMA_CHMAP0_CH4SEL_M    0x000F0000  // uDMA Channel 4 Source Select
#define UDMA_CHMAP0_CH3SEL_M    0x0000F000  // uDMA Channel 3 Source Select
#define UDMA_CHMAP0_CH2SEL_M    0x00000F00  // uDMA Channel 2 Source Select
#define UDMA_CHMAP0_CH1SEL_M    0x000000F0  // uDMA Channel 1 Source Select
#define UDMA_CHMAP0_CH0SEL_M    0x0000000F  // uDMA Channel 0 Source Select
#define UDMA_CHMAP0_CH7SEL_S    28
#define UDMA_CHMAP0_CH6SEL_S    24
#define UDMA_CHMAP0_CH5SEL_S    20
#define UDMA_CHMAP0_CH4SEL_S    16
#define UDMA_CHMAP0_CH3SEL_S    12
#define UDMA_CHMAP0_CH2SEL_S    8
#define UDMA_CHMAP0_CH1SEL_S    4
#define UDMA_CHMAP0_CH0SEL_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP1 register.
//
//*****************************************************************************
#define UDMA_CHMAP1_CH15SEL_M   0xF0000000  // uDMA Channel 15 Source Select
#define UDMA_CHMAP1_CH14SEL_M   0x0F000000  // uDMA Channel 14 Source Select
#define UDMA_CHMAP1_CH13SEL_M   0x00F00000  // uDMA Channel 13 Source Select
#define UDMA_CHMAP1_CH12SEL_M   0x000F0000  // uDMA Channel 12 Source Select
#define UDMA_CHMAP1_CH11SEL_M   0x0000F000  // uDMA Channel 11 Source Select
#define UDMA_CHMAP1_CH10SEL_M   0x00000F00  // uDMA Channel 10 Source Select
#define UDMA_CHMAP1_CH9SEL_M    0x000000F0  // uDMA Channel 9 Source Select
#define UDMA_CHMAP1_CH8SEL_M    0x0000000F  // uDMA Channel 8 Source Select
#define UDMA_CHMAP1_CH15SEL_S   28
#define UDMA_CHMAP1_CH14SEL_S   24
#define UDMA_CHMAP1_CH13SEL_S   20
#define UDMA_CHMAP1_CH12SEL_S   16
#define UDMA_CHMAP1_CH11SEL_S   12
#define UDMA_CHMAP1_CH10SEL_S   8
#define UDMA_CHMAP1_CH9SEL_S    4
#define UDMA_CHMAP1_CH8SEL_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP2 register.
//
//*****************************************************************************
#define UDMA_CHMAP2_CH23SEL_M   0xF0000000  // uDMA Channel 23 Source Select
#define UDMA_CHMAP2_CH22SEL_M   0x0F000000  // uDMA Channel 22 Source Select
#define UDMA_CHMAP2_CH21SEL_M   0x00F00000  // uDMA Channel 21 Source Select
#define UDMA_CHMAP2_CH20SEL_M   0x000F0000  // uDMA Channel 20 Source Select
#define UDMA_CHMAP2_CH19SEL_M   0x0000F000  // uDMA Channel 19 Source Select
#define UDMA_CHMAP2_CH18SEL_M   0x00000F00  // uDMA Channel 18 Source Select
#define UDMA_CHMAP2_CH17SEL_M   0x000000F0  // uDMA Channel 17 Source Select
#define UDMA_CHMAP2_CH16SEL_M   0x0000000F  // uDMA Channel 16 Source Select
#define UDMA_CHMAP2_CH23SEL_S   28
#define UDMA_CHMAP2_CH22SEL_S   24
#define UDMA_CHMAP2_CH21SEL_S   20
#define UDMA_CHMAP2_CH20SEL_S   16
#define UDMA_CHMAP2_CH19SEL_S   12
#define UDMA_CHMAP2_CH18SEL_S   8
#define UDMA_CHMAP2_CH17SEL_S   4
#define UDMA_CHMAP2_CH16SEL_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP3 register.
//
//*****************************************************************************
#define UDMA_CHMAP3_CH31SEL_M   0xF0000000  // uDMA Channel 31 Source Select
#define UDMA_CHMAP3_CH30SEL_M   0x0F000000  // uDMA Channel 30 Source Select
#define UDMA_CHMAP3_CH29SEL_M   0x00F00000  // uDMA Channel 29 Source Select
#define UDMA_CHMAP3_CH28SEL_M   0x000F0000  // uDMA Channel 28 Source Select
#define UDMA_CHMAP3_CH27SEL_M   0x0000F000  // uDMA Channel 27 Source Select
#define UDMA_CHMAP3_CH26SEL_M   0x00000F00  // uDMA Channel 26 Source Select
#define UDMA_CHMAP3_CH25SEL_M   0x000000F0  // uDMA Channel 25 Source Select
#define UDMA_CHMAP3_CH24SEL_M   0x0000000F  // uDMA Channel 24 Source Select
#define UDMA_CHMAP3_CH31SEL_S   28
#define UDMA_CHMAP3_CH30SEL_S   24
#define UDMA_CHMAP3_CH29SEL_S   20
#define UDMA_CHMAP3_CH28SEL_S   16
#define UDMA_CHMAP3_CH27SEL_S   12
#define UDMA_CHMAP3_CH26SEL_S   8
#define UDMA_CHMAP3_CH25SEL_S   4
#define UDMA_CHMAP3_CH24SEL_S   0

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_SRCENDP register.
//
//*****************************************************************************
#define UDMA_SRCENDP_ADDR_M     0xFFFFFFFF  // Source Address End Pointer
#define UDMA_SRCENDP_ADDR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_DSTENDP register.
//
//*****************************************************************************
#define UDMA_DSTENDP_ADDR_M     0xFFFFFFFF  // Destination Address End Pointer
#define UDMA_DSTENDP_ADDR_S     0

//*****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_CHCTL register.
//
//*****************************************************************************
#define UDMA_CHCTL_DSTINC_M     0xC0000000  // Destination Address Increment
#define UDMA_CHCTL_DSTINC_8     0x00000000  // Byte
#define UDMA_CHCTL_DSTINC_16    0x40000000  // Half-word
#define UDMA_CHCTL_DSTINC_32    0x80000000  // Word
#define UDMA_CHCTL_DSTINC_NONE  0xC0000000  // No increment
#define UDMA_CHCTL_DSTSIZE_M    0x30000000  // Destination Data Size
#define UDMA_CHCTL_DSTSIZE_8    0x00000000  // Byte
#define UDMA_CHCTL_DSTSIZE_16   0x10000000  // Half-word
#define UDMA_CHCTL_DSTSIZE_32   0x20000000  // Word
#define UDMA_CHCTL_SRCINC_M     0x0C000000  // Source Address Increment
#define UDMA_CHCTL_SRCINC_8     0x00000000  // Byte
#define UDMA_CHCTL_SRCINC_16    0x04000000  // Half-word
#define UDMA_CHCTL_SRCINC_32    0x08000000  // Word
#define UDMA_CHCTL_SRCINC_NONE  0x0C000000  // No increment
#define UDMA_CHCTL_SRCSIZE_M    0x03000000  // Source Data Size
#define UDMA_CHCTL_SRCSIZE_8    0x00000000  // Byte
#define UDMA_CHCTL_SRCSIZE_16   0x01000000  // Half-word
#define UDMA_CHCTL_SRCSIZE_32   0x02000000  // Word
#define UDMA_CHCTL_ARBSIZE_M    0x0003C000  // Arbitration Size
#define UDMA_CHCTL_ARBSIZE_1    0x00000000  // 1 Transfer
#define UDMA_CHCTL_ARBSIZE_2    0x00004000  // 2 Transfers
#define UDMA_CHCTL_ARBSIZE_4    0x00008000  // 4 Transfers
#define UDMA_CHCTL_ARBSIZE_8    0x0000C000  // 8 Transfers
#define UDMA_CHCTL_ARBSIZE_16   0x00010000  // 16 Transfers
#define UDMA_CHCTL_ARBSIZE_32   0x00014000  // 32 Transfers
#define UDMA_CHCTL_ARBSIZE_64   0x00018000  // 64 Transfers
#define UDMA_CHCTL_ARBSIZE_128  0x0001C000  // 128 Transfers
#define UDMA_CHCTL_ARBSIZE_256  0x00020000  // 256 Transfers
#define UDMA_CHCTL_ARBSIZE_512  0x00024000  // 512 Transfers
#define UDMA_CHCTL_ARBSIZE_1024 0x00028000  // 1024 Transfers
#define UDMA_CHCTL_XFERSIZE_M   0x00003FF0  // Transfer Size (minus 1)
#define UDMA_CHCTL_NXTUSEBURST  0x00000008  // Next Useburst
#define UDMA_CHCTL_XFERMODE_M   0x00000007  // uDMA Transfer Mode
#define UDMA_CHCTL_XFERMODE_STOP                                              \
                                0x00000000  // Stop
#define UDMA_CHCTL_XFERMODE_BASIC                                             \
                                0x00000001  // Basic
#define UDMA_CHCTL_XFERMODE_AUTO                                              \
                                0x00000002  // Auto-Request
#define UDMA_CHCTL_XFERMODE_PINGPONG                                          \
                                0x00000003  // Ping-Pong
#define UDMA_CHCTL_XFERMODE_MEM_SG                                            \
                                0x00000004  // Memory Scatter-Gather
#define UDMA_CHCTL_XFERMODE_MEM_SGA                                           \
                                0x00000005  // Alternate Memory Scatter-Gather
#define UDMA_CHCTL_XFERMODE_PER_SG                                            \
                                0x00000006  // Peripheral Scatter-Gather
#define UDMA_CHCTL_XFERMODE_PER_SGA                                           \
                                0x00000007  // Alternate Peripheral
                                            // Scatter-Gather
#define UDMA_CHCTL_XFERSIZE_S   4

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTLR register.
//
//*****************************************************************************
#define NVIC_ACTLR_DISOOFP      0x00000200  // Disable Out-Of-Order Floating
                                            // Point
#define NVIC_ACTLR_DISFPCA      0x00000100  // Disable CONTROL
#define NVIC_ACTLR_DISFOLD      0x00000004  // Disable IT Folding
#define NVIC_ACTLR_DISWBUF      0x00000002  // Disable Write Buffer
#define NVIC_ACTLR_DISMCYC      0x00000001  // Disable Interrupts of Multiple
                                            // Cycle Instructions

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CTRL register.
//
//*****************************************************************************
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count Flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt Enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_RELOAD register.
//
//*****************************************************************************
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Reload Value
#define NVIC_ST_RELOAD_S        0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CURRENT
// register.
//
//*****************************************************************************
#define NVIC_ST_CURRENT_M       0x00FFFFFF  // Current Value
#define NVIC_ST_CURRENT_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN0 register.
//
//*****************************************************************************
#define NVIC_EN0_INT_M          0xFFFFFFFF  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN1 register.
//
//*****************************************************************************
#define NVIC_EN1_INT_M          0xFFFFFFFF  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN2 register.
//
//*****************************************************************************
#define NVIC_EN2_INT_M          0xFFFFFFFF  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN3 register.
//
//*****************************************************************************
#define NVIC_EN3_INT_M          0xFFFFFFFF  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN4 register.
//
//*****************************************************************************
#define NVIC_EN4_INT_M          0x000007FF  // Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS0 register.
//
//*****************************************************************************
#define NVIC_DIS0_INT_M         0xFFFFFFFF  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS1 register.
//
//*****************************************************************************
#define NVIC_DIS1_INT_M         0xFFFFFFFF  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS2 register.
//
//*****************************************************************************
#define NVIC_DIS2_INT_M         0xFFFFFFFF  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS3 register.
//
//*****************************************************************************
#define NVIC_DIS3_INT_M         0xFFFFFFFF  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS4 register.
//
//*****************************************************************************
#define NVIC_DIS4_INT_M         0x000007FF  // Interrupt Disable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND0 register.
//
//*****************************************************************************
#define NVIC_PEND0_INT_M        0xFFFFFFFF  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND1 register.
//
//*****************************************************************************
#define NVIC_PEND1_INT_M        0xFFFFFFFF  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND2 register.
//
//*****************************************************************************
#define NVIC_PEND2_INT_M        0xFFFFFFFF  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND3 register.
//
//*****************************************************************************
#define NVIC_PEND3_INT_M        0xFFFFFFFF  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND4 register.
//
//*****************************************************************************
#define NVIC_PEND4_INT_M        0x000007FF  // Interrupt Set Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND0 register.
//
//*****************************************************************************
#define NVIC_UNPEND0_INT_M      0xFFFFFFFF  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND1 register.
//
//*****************************************************************************
#define NVIC_UNPEND1_INT_M      0xFFFFFFFF  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND2 register.
//
//*****************************************************************************
#define NVIC_UNPEND2_INT_M      0xFFFFFFFF  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND3 register.
//
//*****************************************************************************
#define NVIC_UNPEND3_INT_M      0xFFFFFFFF  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND4 register.
//
//*****************************************************************************
#define NVIC_UNPEND4_INT_M      0x000007FF  // Interrupt Clear Pending

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE0 register.
//
//*****************************************************************************
#define NVIC_ACTIVE0_INT_M      0xFFFFFFFF  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE1 register.
//
//*****************************************************************************
#define NVIC_ACTIVE1_INT_M      0xFFFFFFFF  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE2 register.
//
//*****************************************************************************
#define NVIC_ACTIVE2_INT_M      0xFFFFFFFF  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE3 register.
//
//*****************************************************************************
#define NVIC_ACTIVE3_INT_M      0xFFFFFFFF  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE4 register.
//
//*****************************************************************************
#define NVIC_ACTIVE4_INT_M      0x000007FF  // Interrupt Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI0 register.
//
//*****************************************************************************
#define NVIC_PRI0_INT3_M        0xE0000000  // Interrupt 3 Priority Mask
#define NVIC_PRI0_INT2_M        0x00E00000  // Interrupt 2 Priority Mask
#define NVIC_PRI0_INT1_M        0x0000E000  // Interrupt 1 Priority Mask
#define NVIC_PRI0_INT0_M        0x000000E0  // Interrupt 0 Priority Mask
#define NVIC_PRI0_INT3_S        29
#define NVIC_PRI0_INT2_S        21
#define NVIC_PRI0_INT1_S        13
#define NVIC_PRI0_INT0_S        5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI1 register.
//
//*****************************************************************************
#define NVIC_PRI1_INT7_M        0xE0000000  // Interrupt 7 Priority Mask
#define NVIC_PRI1_INT6_M        0x00E00000  // Interrupt 6 Priority Mask
#define NVIC_PRI1_INT5_M        0x0000E000  // Interrupt 5 Priority Mask
#define NVIC_PRI1_INT4_M        0x000000E0  // Interrupt 4 Priority Mask
#define NVIC_PRI1_INT7_S        29
#define NVIC_PRI1_INT6_S        21
#define NVIC_PRI1_INT5_S        13
#define NVIC_PRI1_INT4_S        5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI2 register.
//
//*****************************************************************************
#define NVIC_PRI2_INT11_M       0xE0000000  // Interrupt 11 Priority Mask
#define NVIC_PRI2_INT10_M       0x00E00000  // Interrupt 10 Priority Mask
#define NVIC_PRI2_INT9_M        0x0000E000  // Interrupt 9 Priority Mask
#define NVIC_PRI2_INT8_M        0x000000E0  // Interrupt 8 Priority Mask
#define NVIC_PRI2_INT11_S       29
#define NVIC_PRI2_INT10_S       21
#define NVIC_PRI2_INT9_S        13
#define NVIC_PRI2_INT8_S        5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI3 register.
//
//*****************************************************************************
#define NVIC_PRI3_INT15_M       0xE0000000  // Interrupt 15 Priority Mask
#define NVIC_PRI3_INT14_M       0x00E00000  // Interrupt 14 Priority Mask
#define NVIC_PRI3_INT13_M       0x0000E000  // Interrupt 13 Priority Mask
#define NVIC_PRI3_INT12_M       0x000000E0  // Interrupt 12 Priority Mask
#define NVIC_PRI3_INT15_S       29
#define NVIC_PRI3_INT14_S       21
#define NVIC_PRI3_INT13_S       13
#define NVIC_PRI3_INT12_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI4 register.
//
//*****************************************************************************
#define NVIC_PRI4_INT19_M       0xE0000000  // Interrupt 19 Priority Mask
#define NVIC_PRI4_INT18_M       0x00E00000  // Interrupt 18 Priority Mask
#define NVIC_PRI4_INT17_M       0x0000E000  // Interrupt 17 Priority Mask
#define NVIC_PRI4_INT16_M       0x000000E0  // Interrupt 16 Priority Mask
#define NVIC_PRI4_INT19_S       29
#define NVIC_PRI4_INT18_S       21
#define NVIC_PRI4_INT17_S       13
#define NVIC_PRI4_INT16_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI5 register.
//
//*****************************************************************************
#define NVIC_PRI5_INT23_M       0xE0000000  // Interrupt 23 Priority Mask
#define NVIC_PRI5_INT22_M       0x00E00000  // Interrupt 22 Priority Mask
#define NVIC_PRI5_INT21_M       0x0000E000  // Interrupt 21 Priority Mask
#define NVIC_PRI5_INT20_M       0x000000E0  // Interrupt 20 Priority Mask
#define NVIC_PRI5_INT23_S       29
#define NVIC_PRI5_INT22_S       21
#define NVIC_PRI5_INT21_S       13
#define NVIC_PRI5_INT20_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI6 register.
//
//*****************************************************************************
#define NVIC_PRI6_INT27_M       0xE0000000  // Interrupt 27 Priority Mask
#define NVIC_PRI6_INT26_M       0x00E00000  // Interrupt 26 Priority Mask
#define NVIC_PRI6_INT25_M       0x0000E000  // Interrupt 25 Priority Mask
#define NVIC_PRI6_INT24_M       0x000000E0  // Interrupt 24 Priority Mask
#define NVIC_PRI6_INT27_S       29
#define NVIC_PRI6_INT26_S       21
#define NVIC_PRI6_INT25_S       13
#define NVIC_PRI6_INT24_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI7 register.
//
//*****************************************************************************
#define NVIC_PRI7_INT31_M       0xE0000000  // Interrupt 31 Priority Mask
#define NVIC_PRI7_INT30_M       0x00E00000  // Interrupt 30 Priority Mask
#define NVIC_PRI7_INT29_M       0x0000E000  // Interrupt 29 Priority Mask
#define NVIC_PRI7_INT28_M       0x000000E0  // Interrupt 28 Priority Mask
#define NVIC_PRI7_INT31_S       29
#define NVIC_PRI7_INT30_S       21
#define NVIC_PRI7_INT29_S       13
#define NVIC_PRI7_INT28_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI8 register.
//
//*****************************************************************************
#define NVIC_PRI8_INT35_M       0xE0000000  // Interrupt 35 Priority Mask
#define NVIC_PRI8_INT34_M       0x00E00000  // Interrupt 34 Priority Mask
#define NVIC_PRI8_INT33_M       0x0000E000  // Interrupt 33 Priority Mask
#define NVIC_PRI8_INT32_M       0x000000E0  // Interrupt 32 Priority Mask
#define NVIC_PRI8_INT35_S       29
#define NVIC_PRI8_INT34_S       21
#define NVIC_PRI8_INT33_S       13
#define NVIC_PRI8_INT32_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI9 register.
//
//*****************************************************************************
#define NVIC_PRI9_INT39_M       0xE0000000  // Interrupt 39 Priority Mask
#define NVIC_PRI9_INT38_M       0x00E00000  // Interrupt 38 Priority Mask
#define NVIC_PRI9_INT37_M       0x0000E000  // Interrupt 37 Priority Mask
#define NVIC_PRI9_INT36_M       0x000000E0  // Interrupt 36 Priority Mask
#define NVIC_PRI9_INT39_S       29
#define NVIC_PRI9_INT38_S       21
#define NVIC_PRI9_INT37_S       13
#define NVIC_PRI9_INT36_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI10 register.
//
//*****************************************************************************
#define NVIC_PRI10_INT43_M      0xE0000000  // Interrupt 43 Priority Mask
#define NVIC_PRI10_INT42_M      0x00E00000  // Interrupt 42 Priority Mask
#define NVIC_PRI10_INT41_M      0x0000E000  // Interrupt 41 Priority Mask
#define NVIC_PRI10_INT40_M      0x000000E0  // Interrupt 40 Priority Mask
#define NVIC_PRI10_INT43_S      29
#define NVIC_PRI10_INT42_S      21
#define NVIC_PRI10_INT41_S      13
#define NVIC_PRI10_INT40_S      5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI11 register.
//
//*****************************************************************************
#define NVIC_PRI11_INT47_M      0xE0000000  // Interrupt 47 Priority Mask
#define NVIC_PRI11_INT46_M      0x00E00000  // Interrupt 46 Priority Mask
#define NVIC_PRI11_INT45_M      0x0000E000  // Interrupt 45 Priority Mask
#define NVIC_PRI11_INT44_M      0x000000E0  // Interrupt 44 Priority Mask
#define NVIC_PRI11_INT47_S      29
#define NVIC_PRI11_INT46_S      21
#define NVIC_PRI11_INT45_S      13
#define NVIC_PRI11_INT44_S      5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI12 register.
//
//*****************************************************************************
#define NVIC_PRI12_INT51_M      0xE0000000  // Interrupt 51 Priority Mask
#define NVIC_PRI12_INT50_M      0x00E00000  // Interrupt 50 Priority Mask
#define NVIC_PRI12_INT49_M      0x0000E000  // Interrupt 49 Priority Mask
#define NVIC_PRI12_INT48_M      0x000000E0  // Interrupt 48 Priority Mask
#define NVIC_PRI12_INT51_S      29
#define NVIC_PRI12_INT50_S      21
#define NVIC_PRI12_INT49_S      13
#define NVIC_PRI12_INT48_S      5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI13 register.
//
//*****************************************************************************
#define NVIC_PRI13_INT55_M      0xE0000000  // Interrupt 55 Priority Mask
#define NVIC_PRI13_INT54_M      0x00E00000  // Interrupt 54 Priority Mask
#define NVIC_PRI13_INT53_M      0x0000E000  // Interrupt 53 Priority Mask
#define NVIC_PRI13_INT52_M      0x000000E0  // Interrupt 52 Priority Mask
#define NVIC_PRI13_INT55_S      29
#define NVIC_PRI13_INT54_S      21
#define NVIC_PRI13_INT53_S      13
#define NVIC_PRI13_INT52_S      5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI14 register.
//
//*****************************************************************************
#define NVIC_PRI14_INTD_M       0xE0000000  // Interrupt 59 Priority Mask
#define NVIC_PRI14_INTC_M       0x00E00000  // Interrupt 58 Priority Mask
#define NVIC_PRI14_INTB_M       0x0000E000  // Interrupt 57 Priority Mask
#define NVIC_PRI14_INTA_M       0x000000E0  // Interrupt 56 Priority Mask
#define NVIC_PRI14_INTD_S       29
#define NVIC_PRI14_INTC_S       21
#define NVIC_PRI14_INTB_S       13
#define NVIC_PRI14_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI15 register.
//
//*****************************************************************************
#define NVIC_PRI15_INTD_M       0xE0000000  // Interrupt 63 Priority Mask
#define NVIC_PRI15_INTC_M       0x00E00000  // Interrupt 62 Priority Mask
#define NVIC_PRI15_INTB_M       0x0000E000  // Interrupt 61 Priority Mask
#define NVIC_PRI15_INTA_M       0x000000E0  // Interrupt 60 Priority Mask
#define NVIC_PRI15_INTD_S       29
#define NVIC_PRI15_INTC_S       21
#define NVIC_PRI15_INTB_S       13
#define NVIC_PRI15_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI16 register.
//
//*****************************************************************************
#define NVIC_PRI16_INTD_M       0xE0000000  // Interrupt 67 Priority Mask
#define NVIC_PRI16_INTC_M       0x00E00000  // Interrupt 66 Priority Mask
#define NVIC_PRI16_INTB_M       0x0000E000  // Interrupt 65 Priority Mask
#define NVIC_PRI16_INTA_M       0x000000E0  // Interrupt 64 Priority Mask
#define NVIC_PRI16_INTD_S       29
#define NVIC_PRI16_INTC_S       21
#define NVIC_PRI16_INTB_S       13
#define NVIC_PRI16_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI17 register.
//
//*****************************************************************************
#define NVIC_PRI17_INTD_M       0xE0000000  // Interrupt 71 Priority Mask
#define NVIC_PRI17_INTC_M       0x00E00000  // Interrupt 70 Priority Mask
#define NVIC_PRI17_INTB_M       0x0000E000  // Interrupt 69 Priority Mask
#define NVIC_PRI17_INTA_M       0x000000E0  // Interrupt 68 Priority Mask
#define NVIC_PRI17_INTD_S       29
#define NVIC_PRI17_INTC_S       21
#define NVIC_PRI17_INTB_S       13
#define NVIC_PRI17_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI18 register.
//
//*****************************************************************************
#define NVIC_PRI18_INTD_M       0xE0000000  // Interrupt 75 Priority Mask
#define NVIC_PRI18_INTC_M       0x00E00000  // Interrupt 74 Priority Mask
#define NVIC_PRI18_INTB_M       0x0000E000  // Interrupt 73 Priority Mask
#define NVIC_PRI18_INTA_M       0x000000E0  // Interrupt 72 Priority Mask
#define NVIC_PRI18_INTD_S       29
#define NVIC_PRI18_INTC_S       21
#define NVIC_PRI18_INTB_S       13
#define NVIC_PRI18_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI19 register.
//
//*****************************************************************************
#define NVIC_PRI19_INTD_M       0xE0000000  // Interrupt 79 Priority Mask
#define NVIC_PRI19_INTC_M       0x00E00000  // Interrupt 78 Priority Mask
#define NVIC_PRI19_INTB_M       0x0000E000  // Interrupt 77 Priority Mask
#define NVIC_PRI19_INTA_M       0x000000E0  // Interrupt 76 Priority Mask
#define NVIC_PRI19_INTD_S       29
#define NVIC_PRI19_INTC_S       21
#define NVIC_PRI19_INTB_S       13
#define NVIC_PRI19_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI20 register.
//
//*****************************************************************************
#define NVIC_PRI20_INTD_M       0xE0000000  // Interrupt 83 Priority Mask
#define NVIC_PRI20_INTC_M       0x00E00000  // Interrupt 82 Priority Mask
#define NVIC_PRI20_INTB_M       0x0000E000  // Interrupt 81 Priority Mask
#define NVIC_PRI20_INTA_M       0x000000E0  // Interrupt 80 Priority Mask
#define NVIC_PRI20_INTD_S       29
#define NVIC_PRI20_INTC_S       21
#define NVIC_PRI20_INTB_S       13
#define NVIC_PRI20_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI21 register.
//
//*****************************************************************************
#define NVIC_PRI21_INTD_M       0xE0000000  // Interrupt 87 Priority Mask
#define NVIC_PRI21_INTC_M       0x00E00000  // Interrupt 86 Priority Mask
#define NVIC_PRI21_INTB_M       0x0000E000  // Interrupt 85 Priority Mask
#define NVIC_PRI21_INTA_M       0x000000E0  // Interrupt 84 Priority Mask
#define NVIC_PRI21_INTD_S       29
#define NVIC_PRI21_INTC_S       21
#define NVIC_PRI21_INTB_S       13
#define NVIC_PRI21_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI22 register.
//
//*****************************************************************************
#define NVIC_PRI22_INTD_M       0xE0000000  // Interrupt 91 Priority Mask
#define NVIC_PRI22_INTC_M       0x00E00000  // Interrupt 90 Priority Mask
#define NVIC_PRI22_INTB_M       0x0000E000  // Interrupt 89 Priority Mask
#define NVIC_PRI22_INTA_M       0x000000E0  // Interrupt 88 Priority Mask
#define NVIC_PRI22_INTD_S       29
#define NVIC_PRI22_INTC_S       21
#define NVIC_PRI22_INTB_S       13
#define NVIC_PRI22_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI23 register.
//
//*****************************************************************************
#define NVIC_PRI23_INTD_M       0xE0000000  // Interrupt 95 Priority Mask
#define NVIC_PRI23_INTC_M       0x00E00000  // Interrupt 94 Priority Mask
#define NVIC_PRI23_INTB_M       0x0000E000  // Interrupt 93 Priority Mask
#define NVIC_PRI23_INTA_M       0x000000E0  // Interrupt 92 Priority Mask
#define NVIC_PRI23_INTD_S       29
#define NVIC_PRI23_INTC_S       21
#define NVIC_PRI23_INTB_S       13
#define NVIC_PRI23_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI24 register.
//
//*****************************************************************************
#define NVIC_PRI24_INTD_M       0xE0000000  // Interrupt 99 Priority Mask
#define NVIC_PRI24_INTC_M       0x00E00000  // Interrupt 98 Priority Mask
#define NVIC_PRI24_INTB_M       0x0000E000  // Interrupt 97 Priority Mask
#define NVIC_PRI24_INTA_M       0x000000E0  // Interrupt 96 Priority Mask
#define NVIC_PRI24_INTD_S       29
#define NVIC_PRI24_INTC_S       21
#define NVIC_PRI24_INTB_S       13
#define NVIC_PRI24_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI25 register.
//
//*****************************************************************************
#define NVIC_PRI25_INTD_M       0xE0000000  // Interrupt 103 Priority Mask
#define NVIC_PRI25_INTC_M       0x00E00000  // Interrupt 102 Priority Mask
#define NVIC_PRI25_INTB_M       0x0000E000  // Interrupt 101 Priority Mask
#define NVIC_PRI25_INTA_M       0x000000E0  // Interrupt 100 Priority Mask
#define NVIC_PRI25_INTD_S       29
#define NVIC_PRI25_INTC_S       21
#define NVIC_PRI25_INTB_S       13
#define NVIC_PRI25_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI26 register.
//
//*****************************************************************************
#define NVIC_PRI26_INTD_M       0xE0000000  // Interrupt 107 Priority Mask
#define NVIC_PRI26_INTC_M       0x00E00000  // Interrupt 106 Priority Mask
#define NVIC_PRI26_INTB_M       0x0000E000  // Interrupt 105 Priority Mask
#define NVIC_PRI26_INTA_M       0x000000E0  // Interrupt 104 Priority Mask
#define NVIC_PRI26_INTD_S       29
#define NVIC_PRI26_INTC_S       21
#define NVIC_PRI26_INTB_S       13
#define NVIC_PRI26_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI27 register.
//
//*****************************************************************************
#define NVIC_PRI27_INTD_M       0xE0000000  // Interrupt 111 Priority Mask
#define NVIC_PRI27_INTC_M       0x00E00000  // Interrupt 110 Priority Mask
#define NVIC_PRI27_INTB_M       0x0000E000  // Interrupt 109 Priority Mask
#define NVIC_PRI27_INTA_M       0x000000E0  // Interrupt 108 Priority Mask
#define NVIC_PRI27_INTD_S       29
#define NVIC_PRI27_INTC_S       21
#define NVIC_PRI27_INTB_S       13
#define NVIC_PRI27_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI28 register.
//
//*****************************************************************************
#define NVIC_PRI28_INTD_M       0xE0000000  // Interrupt 115 Priority Mask
#define NVIC_PRI28_INTC_M       0x00E00000  // Interrupt 114 Priority Mask
#define NVIC_PRI28_INTB_M       0x0000E000  // Interrupt 113 Priority Mask
#define NVIC_PRI28_INTA_M       0x000000E0  // Interrupt 112 Priority Mask
#define NVIC_PRI28_INTD_S       29
#define NVIC_PRI28_INTC_S       21
#define NVIC_PRI28_INTB_S       13
#define NVIC_PRI28_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI29 register.
//
//*****************************************************************************
#define NVIC_PRI29_INTD_M       0xE0000000  // Interrupt 119 Priority Mask
#define NVIC_PRI29_INTC_M       0x00E00000  // Interrupt 118 Priority Mask
#define NVIC_PRI29_INTB_M       0x0000E000  // Interrupt 117 Priority Mask
#define NVIC_PRI29_INTA_M       0x000000E0  // Interrupt 116 Priority Mask
#define NVIC_PRI29_INTD_S       29
#define NVIC_PRI29_INTC_S       21
#define NVIC_PRI29_INTB_S       13
#define NVIC_PRI29_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI30 register.
//
//*****************************************************************************
#define NVIC_PRI30_INTD_M       0xE0000000  // Interrupt 123 Priority Mask
#define NVIC_PRI30_INTC_M       0x00E00000  // Interrupt 122 Priority Mask
#define NVIC_PRI30_INTB_M       0x0000E000  // Interrupt 121 Priority Mask
#define NVIC_PRI30_INTA_M       0x000000E0  // Interrupt 120 Priority Mask
#define NVIC_PRI30_INTD_S       29
#define NVIC_PRI30_INTC_S       21
#define NVIC_PRI30_INTB_S       13
#define NVIC_PRI30_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI31 register.
//
//*****************************************************************************
#define NVIC_PRI31_INTD_M       0xE0000000  // Interrupt 127 Priority Mask
#define NVIC_PRI31_INTC_M       0x00E00000  // Interrupt 126 Priority Mask
#define NVIC_PRI31_INTB_M       0x0000E000  // Interrupt 125 Priority Mask
#define NVIC_PRI31_INTA_M       0x000000E0  // Interrupt 124 Priority Mask
#define NVIC_PRI31_INTD_S       29
#define NVIC_PRI31_INTC_S       21
#define NVIC_PRI31_INTB_S       13
#define NVIC_PRI31_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI32 register.
//
//*****************************************************************************
#define NVIC_PRI32_INTD_M       0xE0000000  // Interrupt 131 Priority Mask
#define NVIC_PRI32_INTC_M       0x00E00000  // Interrupt 130 Priority Mask
#define NVIC_PRI32_INTB_M       0x0000E000  // Interrupt 129 Priority Mask
#define NVIC_PRI32_INTA_M       0x000000E0  // Interrupt 128 Priority Mask
#define NVIC_PRI32_INTD_S       29
#define NVIC_PRI32_INTC_S       21
#define NVIC_PRI32_INTB_S       13
#define NVIC_PRI32_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI33 register.
//
//*****************************************************************************
#define NVIC_PRI33_INTD_M       0xE0000000  // Interrupt Priority for Interrupt
                                            // [4n+3]
#define NVIC_PRI33_INTC_M       0x00E00000  // Interrupt Priority for Interrupt
                                            // [4n+2]
#define NVIC_PRI33_INTB_M       0x0000E000  // Interrupt Priority for Interrupt
                                            // [4n+1]
#define NVIC_PRI33_INTA_M       0x000000E0  // Interrupt Priority for Interrupt
                                            // [4n]
#define NVIC_PRI33_INTD_S       29
#define NVIC_PRI33_INTC_S       21
#define NVIC_PRI33_INTB_S       13
#define NVIC_PRI33_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI34 register.
//
//*****************************************************************************
#define NVIC_PRI34_INTD_M       0xE0000000  // Interrupt Priority for Interrupt
                                            // [4n+3]
#define NVIC_PRI34_INTC_M       0x00E00000  // Interrupt Priority for Interrupt
                                            // [4n+2]
#define NVIC_PRI34_INTB_M       0x0000E000  // Interrupt Priority for Interrupt
                                            // [4n+1]
#define NVIC_PRI34_INTA_M       0x000000E0  // Interrupt Priority for Interrupt
                                            // [4n]
#define NVIC_PRI34_INTD_S       29
#define NVIC_PRI34_INTC_S       21
#define NVIC_PRI34_INTB_S       13
#define NVIC_PRI34_INTA_S       5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPUID register.
//
//*****************************************************************************
#define NVIC_CPUID_IMP_M        0xFF000000  // Implementer Code
#define NVIC_CPUID_IMP_ARM      0x41000000  // ARM
#define NVIC_CPUID_VAR_M        0x00F00000  // Variant Number
#define NVIC_CPUID_CON_M        0x000F0000  // Constant
#define NVIC_CPUID_PARTNO_M     0x0000FFF0  // Part Number
#define NVIC_CPUID_PARTNO_CM4   0x0000C240  // Cortex-M4 processor
#define NVIC_CPUID_REV_M        0x0000000F  // Revision Number

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_INT_CTRL register.
//
//*****************************************************************************
#define NVIC_INT_CTRL_NMI_SET   0x80000000  // NMI Set Pending
#define NVIC_INT_CTRL_PEND_SV   0x10000000  // PendSV Set Pending
#define NVIC_INT_CTRL_UNPEND_SV 0x08000000  // PendSV Clear Pending
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // SysTick Set Pending
#define NVIC_INT_CTRL_PENDSTCLR 0x02000000  // SysTick Clear Pending
#define NVIC_INT_CTRL_ISR_PRE   0x00800000  // Debug Interrupt Handling
#define NVIC_INT_CTRL_ISR_PEND  0x00400000  // Interrupt Pending
#define NVIC_INT_CTRL_VEC_PEN_M 0x000FF000  // Interrupt Pending Vector Number
#define NVIC_INT_CTRL_VEC_PEN_NMI                                             \
                                0x00002000  // NMI
#define NVIC_INT_CTRL_VEC_PEN_HARD                                            \
                                0x00003000  // Hard fault
#define NVIC_INT_CTRL_VEC_PEN_MEM                                             \
                                0x00004000  // Memory management fault
#define NVIC_INT_CTRL_VEC_PEN_BUS                                             \
                                0x00005000  // Bus fault
#define NVIC_INT_CTRL_VEC_PEN_USG                                             \
                                0x00006000  // Usage fault
#define NVIC_INT_CTRL_VEC_PEN_SVC                                             \
                                0x0000B000  // SVCall
#define NVIC_INT_CTRL_VEC_PEN_PNDSV                                           \
                                0x0000E000  // PendSV
#define NVIC_INT_CTRL_VEC_PEN_TICK                                            \
                                0x0000F000  // SysTick
#define NVIC_INT_CTRL_RET_BASE  0x00000800  // Return to Base
#define NVIC_INT_CTRL_VEC_ACT_M 0x000000FF  // Interrupt Pending Vector Number
#define NVIC_INT_CTRL_VEC_ACT_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_VTABLE register.
//
//*****************************************************************************
#define NVIC_VTABLE_OFFSET_M    0xFFFFFC00  // Vector Table Offset
#define NVIC_VTABLE_OFFSET_S    10

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_APINT register.
//
//*****************************************************************************
#define NVIC_APINT_VECTKEY_M    0xFFFF0000  // Register Key
#define NVIC_APINT_VECTKEY      0x05FA0000  // Vector key
#define NVIC_APINT_ENDIANESS    0x00008000  // Data Endianess
#define NVIC_APINT_PRIGROUP_M   0x00000700  // Interrupt Priority Grouping
#define NVIC_APINT_PRIGROUP_7_1 0x00000000  // Priority group 7.1 split
#define NVIC_APINT_PRIGROUP_6_2 0x00000100  // Priority group 6.2 split
#define NVIC_APINT_PRIGROUP_5_3 0x00000200  // Priority group 5.3 split
#define NVIC_APINT_PRIGROUP_4_4 0x00000300  // Priority group 4.4 split
#define NVIC_APINT_PRIGROUP_3_5 0x00000400  // Priority group 3.5 split
#define NVIC_APINT_PRIGROUP_2_6 0x00000500  // Priority group 2.6 split
#define NVIC_APINT_PRIGROUP_1_7 0x00000600  // Priority group 1.7 split
#define NVIC_APINT_PRIGROUP_0_8 0x00000700  // Priority group 0.8 split
#define NVIC_APINT_SYSRESETREQ  0x00000004  // System Reset Request
#define NVIC_APINT_VECT_CLR_ACT 0x00000002  // Clear Active NMI / Fault
#define NVIC_APINT_VECT_RESET   0x00000001  // System Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_CTRL register.
//
//*****************************************************************************
#define NVIC_SYS_CTRL_SEVONPEND 0x00000010  // Wake Up on Pending
#define NVIC_SYS_CTRL_SLEEPDEEP 0x00000004  // Deep Sleep Enable
#define NVIC_SYS_CTRL_SLEEPEXIT 0x00000002  // Sleep on ISR Exit

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CFG_CTRL register.
//
//*****************************************************************************
#define NVIC_CFG_CTRL_STKALIGN  0x00000200  // Stack Alignment on Exception
                                            // Entry
#define NVIC_CFG_CTRL_BFHFNMIGN 0x00000100  // Ignore Bus Fault in NMI and
                                            // Fault
#define NVIC_CFG_CTRL_DIV0      0x00000010  // Trap on Divide by 0
#define NVIC_CFG_CTRL_UNALIGNED 0x00000008  // Trap on Unaligned Access
#define NVIC_CFG_CTRL_MAIN_PEND 0x00000002  // Allow Main Interrupt Trigger
#define NVIC_CFG_CTRL_BASE_THR  0x00000001  // Thread State Control

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI1 register.
//
//*****************************************************************************
#define NVIC_SYS_PRI1_USAGE_M   0x00E00000  // Usage Fault Priority
#define NVIC_SYS_PRI1_BUS_M     0x0000E000  // Bus Fault Priority
#define NVIC_SYS_PRI1_MEM_M     0x000000E0  // Memory Management Fault Priority
#define NVIC_SYS_PRI1_USAGE_S   21
#define NVIC_SYS_PRI1_BUS_S     13
#define NVIC_SYS_PRI1_MEM_S     5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI2 register.
//
//*****************************************************************************
#define NVIC_SYS_PRI2_SVC_M     0xE0000000  // SVCall Priority
#define NVIC_SYS_PRI2_SVC_S     29

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI3 register.
//
//*****************************************************************************
#define NVIC_SYS_PRI3_TICK_M    0xE0000000  // SysTick Exception Priority
#define NVIC_SYS_PRI3_PENDSV_M  0x00E00000  // PendSV Priority
#define NVIC_SYS_PRI3_DEBUG_M   0x000000E0  // Debug Priority
#define NVIC_SYS_PRI3_TICK_S    29
#define NVIC_SYS_PRI3_PENDSV_S  21
#define NVIC_SYS_PRI3_DEBUG_S   5

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_HND_CTRL
// register.
//
//*****************************************************************************
#define NVIC_SYS_HND_CTRL_USAGE 0x00040000  // Usage Fault Enable
#define NVIC_SYS_HND_CTRL_BUS   0x00020000  // Bus Fault Enable
#define NVIC_SYS_HND_CTRL_MEM   0x00010000  // Memory Management Fault Enable
#define NVIC_SYS_HND_CTRL_SVC   0x00008000  // SVC Call Pending
#define NVIC_SYS_HND_CTRL_BUSP  0x00004000  // Bus Fault Pending
#define NVIC_SYS_HND_CTRL_MEMP  0x00002000  // Memory Management Fault Pending
#define NVIC_SYS_HND_CTRL_USAGEP                                              \
                                0x00001000  // Usage Fault Pending
#define NVIC_SYS_HND_CTRL_TICK  0x00000800  // SysTick Exception Active
#define NVIC_SYS_HND_CTRL_PNDSV 0x00000400  // PendSV Exception Active
#define NVIC_SYS_HND_CTRL_MON   0x00000100  // Debug Monitor Active
#define NVIC_SYS_HND_CTRL_SVCA  0x00000080  // SVC Call Active
#define NVIC_SYS_HND_CTRL_USGA  0x00000008  // Usage Fault Active
#define NVIC_SYS_HND_CTRL_BUSA  0x00000002  // Bus Fault Active
#define NVIC_SYS_HND_CTRL_MEMA  0x00000001  // Memory Management Fault Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_STAT
// register.
//
//*****************************************************************************
#define NVIC_FAULT_STAT_DIV0    0x02000000  // Divide-by-Zero Usage Fault
#define NVIC_FAULT_STAT_UNALIGN 0x01000000  // Unaligned Access Usage Fault
#define NVIC_FAULT_STAT_NOCP    0x00080000  // No Coprocessor Usage Fault
#define NVIC_FAULT_STAT_INVPC   0x00040000  // Invalid PC Load Usage Fault
#define NVIC_FAULT_STAT_INVSTAT 0x00020000  // Invalid State Usage Fault
#define NVIC_FAULT_STAT_UNDEF   0x00010000  // Undefined Instruction Usage
                                            // Fault
#define NVIC_FAULT_STAT_BFARV   0x00008000  // Bus Fault Address Register Valid
#define NVIC_FAULT_STAT_BLSPERR 0x00002000  // Bus Fault on Floating-Point Lazy
                                            // State Preservation
#define NVIC_FAULT_STAT_BSTKE   0x00001000  // Stack Bus Fault
#define NVIC_FAULT_STAT_BUSTKE  0x00000800  // Unstack Bus Fault
#define NVIC_FAULT_STAT_IMPRE   0x00000400  // Imprecise Data Bus Error
#define NVIC_FAULT_STAT_PRECISE 0x00000200  // Precise Data Bus Error
#define NVIC_FAULT_STAT_IBUS    0x00000100  // Instruction Bus Error
#define NVIC_FAULT_STAT_MMARV   0x00000080  // Memory Management Fault Address
                                            // Register Valid
#define NVIC_FAULT_STAT_MLSPERR 0x00000020  // Memory Management Fault on
                                            // Floating-Point Lazy State
                                            // Preservation
#define NVIC_FAULT_STAT_MSTKE   0x00000010  // Stack Access Violation
#define NVIC_FAULT_STAT_MUSTKE  0x00000008  // Unstack Access Violation
#define NVIC_FAULT_STAT_DERR    0x00000002  // Data Access Violation
#define NVIC_FAULT_STAT_IERR    0x00000001  // Instruction Access Violation

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_HFAULT_STAT
// register.
//
//*****************************************************************************
#define NVIC_HFAULT_STAT_DBG    0x80000000  // Debug Event
#define NVIC_HFAULT_STAT_FORCED 0x40000000  // Forced Hard Fault
#define NVIC_HFAULT_STAT_VECT   0x00000002  // Vector Table Read Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DEBUG_STAT
// register.
//
//*****************************************************************************
#define NVIC_DEBUG_STAT_EXTRNL  0x00000010  // EDBGRQ asserted
#define NVIC_DEBUG_STAT_VCATCH  0x00000008  // Vector catch
#define NVIC_DEBUG_STAT_DWTTRAP 0x00000004  // DWT match
#define NVIC_DEBUG_STAT_BKPT    0x00000002  // Breakpoint instruction
#define NVIC_DEBUG_STAT_HALTED  0x00000001  // Halt request

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MM_ADDR register.
//
//*****************************************************************************
#define NVIC_MM_ADDR_M          0xFFFFFFFF  // Fault Address
#define NVIC_MM_ADDR_S          0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_ADDR
// register.
//
//*****************************************************************************
#define NVIC_FAULT_ADDR_M       0xFFFFFFFF  // Fault Address
#define NVIC_FAULT_ADDR_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPAC register.
//
//*****************************************************************************
#define NVIC_CPAC_CP11_M        0x00C00000  // CP11 Coprocessor Access
                                            // Privilege
#define NVIC_CPAC_CP11_DIS      0x00000000  // Access Denied
#define NVIC_CPAC_CP11_PRIV     0x00400000  // Privileged Access Only
#define NVIC_CPAC_CP11_FULL     0x00C00000  // Full Access
#define NVIC_CPAC_CP10_M        0x00300000  // CP10 Coprocessor Access
                                            // Privilege
#define NVIC_CPAC_CP10_DIS      0x00000000  // Access Denied
#define NVIC_CPAC_CP10_PRIV     0x00100000  // Privileged Access Only
#define NVIC_CPAC_CP10_FULL     0x00300000  // Full Access

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_TYPE register.
//
//*****************************************************************************
#define NVIC_MPU_TYPE_IREGION_M 0x00FF0000  // Number of I Regions
#define NVIC_MPU_TYPE_DREGION_M 0x0000FF00  // Number of D Regions
#define NVIC_MPU_TYPE_SEPARATE  0x00000001  // Separate or Unified MPU
#define NVIC_MPU_TYPE_IREGION_S 16
#define NVIC_MPU_TYPE_DREGION_S 8

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_CTRL register.
//
//*****************************************************************************
#define NVIC_MPU_CTRL_PRIVDEFEN 0x00000004  // MPU Default Region
#define NVIC_MPU_CTRL_HFNMIENA  0x00000002  // MPU Enabled During Faults
#define NVIC_MPU_CTRL_ENABLE    0x00000001  // MPU Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_NUMBER
// register.
//
//*****************************************************************************
#define NVIC_MPU_NUMBER_M       0x00000007  // MPU Region to Access
#define NVIC_MPU_NUMBER_S       0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE register.
//
//*****************************************************************************
#define NVIC_MPU_BASE_ADDR_M    0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_BASE_VALID     0x00000010  // Region Number Valid
#define NVIC_MPU_BASE_REGION_M  0x00000007  // Region Number
#define NVIC_MPU_BASE_ADDR_S    5
#define NVIC_MPU_BASE_REGION_S  0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR register.
//
//*****************************************************************************
#define NVIC_MPU_ATTR_XN        0x10000000  // Instruction Access Disable
#define NVIC_MPU_ATTR_AP_M      0x07000000  // Access Privilege
#define NVIC_MPU_ATTR_TEX_M     0x00380000  // Type Extension Mask
#define NVIC_MPU_ATTR_SHAREABLE 0x00040000  // Shareable
#define NVIC_MPU_ATTR_CACHEABLE 0x00020000  // Cacheable
#define NVIC_MPU_ATTR_BUFFRABLE 0x00010000  // Bufferable
#define NVIC_MPU_ATTR_SRD_M     0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_ATTR_SIZE_M    0x0000003E  // Region Size Mask
#define NVIC_MPU_ATTR_ENABLE    0x00000001  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE1 register.
//
//*****************************************************************************
#define NVIC_MPU_BASE1_ADDR_M   0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_BASE1_VALID    0x00000010  // Region Number Valid
#define NVIC_MPU_BASE1_REGION_M 0x00000007  // Region Number
#define NVIC_MPU_BASE1_ADDR_S   5
#define NVIC_MPU_BASE1_REGION_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR1 register.
//
//*****************************************************************************
#define NVIC_MPU_ATTR1_XN       0x10000000  // Instruction Access Disable
#define NVIC_MPU_ATTR1_AP_M     0x07000000  // Access Privilege
#define NVIC_MPU_ATTR1_TEX_M    0x00380000  // Type Extension Mask
#define NVIC_MPU_ATTR1_SHAREABLE                                              \
                                0x00040000  // Shareable
#define NVIC_MPU_ATTR1_CACHEABLE                                              \
                                0x00020000  // Cacheable
#define NVIC_MPU_ATTR1_BUFFRABLE                                              \
                                0x00010000  // Bufferable
#define NVIC_MPU_ATTR1_SRD_M    0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_ATTR1_SIZE_M   0x0000003E  // Region Size Mask
#define NVIC_MPU_ATTR1_ENABLE   0x00000001  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE2 register.
//
//*****************************************************************************
#define NVIC_MPU_BASE2_ADDR_M   0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_BASE2_VALID    0x00000010  // Region Number Valid
#define NVIC_MPU_BASE2_REGION_M 0x00000007  // Region Number
#define NVIC_MPU_BASE2_ADDR_S   5
#define NVIC_MPU_BASE2_REGION_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR2 register.
//
//*****************************************************************************
#define NVIC_MPU_ATTR2_XN       0x10000000  // Instruction Access Disable
#define NVIC_MPU_ATTR2_AP_M     0x07000000  // Access Privilege
#define NVIC_MPU_ATTR2_TEX_M    0x00380000  // Type Extension Mask
#define NVIC_MPU_ATTR2_SHAREABLE                                              \
                                0x00040000  // Shareable
#define NVIC_MPU_ATTR2_CACHEABLE                                              \
                                0x00020000  // Cacheable
#define NVIC_MPU_ATTR2_BUFFRABLE                                              \
                                0x00010000  // Bufferable
#define NVIC_MPU_ATTR2_SRD_M    0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_ATTR2_SIZE_M   0x0000003E  // Region Size Mask
#define NVIC_MPU_ATTR2_ENABLE   0x00000001  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE3 register.
//
//*****************************************************************************
#define NVIC_MPU_BASE3_ADDR_M   0xFFFFFFE0  // Base Address Mask
#define NVIC_MPU_BASE3_VALID    0x00000010  // Region Number Valid
#define NVIC_MPU_BASE3_REGION_M 0x00000007  // Region Number
#define NVIC_MPU_BASE3_ADDR_S   5
#define NVIC_MPU_BASE3_REGION_S 0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR3 register.
//
//*****************************************************************************
#define NVIC_MPU_ATTR3_XN       0x10000000  // Instruction Access Disable
#define NVIC_MPU_ATTR3_AP_M     0x07000000  // Access Privilege
#define NVIC_MPU_ATTR3_TEX_M    0x00380000  // Type Extension Mask
#define NVIC_MPU_ATTR3_SHAREABLE                                              \
                                0x00040000  // Shareable
#define NVIC_MPU_ATTR3_CACHEABLE                                              \
                                0x00020000  // Cacheable
#define NVIC_MPU_ATTR3_BUFFRABLE                                              \
                                0x00010000  // Bufferable
#define NVIC_MPU_ATTR3_SRD_M    0x0000FF00  // Subregion Disable Bits
#define NVIC_MPU_ATTR3_SIZE_M   0x0000003E  // Region Size Mask
#define NVIC_MPU_ATTR3_ENABLE   0x00000001  // Region Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_CTRL register.
//
//*****************************************************************************
#define NVIC_DBG_CTRL_DBGKEY_M  0xFFFF0000  // Debug key mask
#define NVIC_DBG_CTRL_DBGKEY    0xA05F0000  // Debug key
#define NVIC_DBG_CTRL_S_RESET_ST                                              \
                                0x02000000  // Core has reset since last read
#define NVIC_DBG_CTRL_S_RETIRE_ST                                             \
                                0x01000000  // Core has executed insruction
                                            // since last read
#define NVIC_DBG_CTRL_S_LOCKUP  0x00080000  // Core is locked up
#define NVIC_DBG_CTRL_S_SLEEP   0x00040000  // Core is sleeping
#define NVIC_DBG_CTRL_S_HALT    0x00020000  // Core status on halt
#define NVIC_DBG_CTRL_S_REGRDY  0x00010000  // Register read/write available
#define NVIC_DBG_CTRL_C_SNAPSTALL                                             \
                                0x00000020  // Breaks a stalled load/store
#define NVIC_DBG_CTRL_C_MASKINT 0x00000008  // Mask interrupts when stepping
#define NVIC_DBG_CTRL_C_STEP    0x00000004  // Step the core
#define NVIC_DBG_CTRL_C_HALT    0x00000002  // Halt the core
#define NVIC_DBG_CTRL_C_DEBUGEN 0x00000001  // Enable debug

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_XFER register.
//
//*****************************************************************************
#define NVIC_DBG_XFER_REG_WNR   0x00010000  // Write or not read
#define NVIC_DBG_XFER_REG_SEL_M 0x0000001F  // Register
#define NVIC_DBG_XFER_REG_R0    0x00000000  // Register R0
#define NVIC_DBG_XFER_REG_R1    0x00000001  // Register R1
#define NVIC_DBG_XFER_REG_R2    0x00000002  // Register R2
#define NVIC_DBG_XFER_REG_R3    0x00000003  // Register R3
#define NVIC_DBG_XFER_REG_R4    0x00000004  // Register R4
#define NVIC_DBG_XFER_REG_R5    0x00000005  // Register R5
#define NVIC_DBG_XFER_REG_R6    0x00000006  // Register R6
#define NVIC_DBG_XFER_REG_R7    0x00000007  // Register R7
#define NVIC_DBG_XFER_REG_R8    0x00000008  // Register R8
#define NVIC_DBG_XFER_REG_R9    0x00000009  // Register R9
#define NVIC_DBG_XFER_REG_R10   0x0000000A  // Register R10
#define NVIC_DBG_XFER_REG_R11   0x0000000B  // Register R11
#define NVIC_DBG_XFER_REG_R12   0x0000000C  // Register R12
#define NVIC_DBG_XFER_REG_R13   0x0000000D  // Register R13
#define NVIC_DBG_XFER_REG_R14   0x0000000E  // Register R14
#define NVIC_DBG_XFER_REG_R15   0x0000000F  // Register R15
#define NVIC_DBG_XFER_REG_FLAGS 0x00000010  // xPSR/Flags register
#define NVIC_DBG_XFER_REG_MSP   0x00000011  // Main SP
#define NVIC_DBG_XFER_REG_PSP   0x00000012  // Process SP
#define NVIC_DBG_XFER_REG_DSP   0x00000013  // Deep SP
#define NVIC_DBG_XFER_REG_CFBP  0x00000014  // Control/Fault/BasePri/PriMask

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_DATA register.
//
//*****************************************************************************
#define NVIC_DBG_DATA_M         0xFFFFFFFF  // Data temporary cache
#define NVIC_DBG_DATA_S         0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_INT register.
//
//*****************************************************************************
#define NVIC_DBG_INT_HARDERR    0x00000400  // Debug trap on hard fault
#define NVIC_DBG_INT_INTERR     0x00000200  // Debug trap on interrupt errors
#define NVIC_DBG_INT_BUSERR     0x00000100  // Debug trap on bus error
#define NVIC_DBG_INT_STATERR    0x00000080  // Debug trap on usage fault state
#define NVIC_DBG_INT_CHKERR     0x00000040  // Debug trap on usage fault check
#define NVIC_DBG_INT_NOCPERR    0x00000020  // Debug trap on coprocessor error
#define NVIC_DBG_INT_MMERR      0x00000010  // Debug trap on mem manage fault
#define NVIC_DBG_INT_RESET      0x00000008  // Core reset status
#define NVIC_DBG_INT_RSTPENDCLR 0x00000004  // Clear pending core reset
#define NVIC_DBG_INT_RSTPENDING 0x00000002  // Core reset is pending
#define NVIC_DBG_INT_RSTVCATCH  0x00000001  // Reset vector catch

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SW_TRIG register.
//
//*****************************************************************************
#define NVIC_SW_TRIG_INTID_M    0x000000FF  // Interrupt ID
#define NVIC_SW_TRIG_INTID_S    0

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPCC register.
//
//*****************************************************************************
#define NVIC_FPCC_ASPEN         0x80000000  // Automatic State Preservation
                                            // Enable
#define NVIC_FPCC_LSPEN         0x40000000  // Lazy State Preservation Enable
#define NVIC_FPCC_MONRDY        0x00000100  // Monitor Ready
#define NVIC_FPCC_BFRDY         0x00000040  // Bus Fault Ready
#define NVIC_FPCC_MMRDY         0x00000020  // Memory Management Fault Ready
#define NVIC_FPCC_HFRDY         0x00000010  // Hard Fault Ready
#define NVIC_FPCC_THREAD        0x00000008  // Thread Mode
#define NVIC_FPCC_USER          0x00000002  // User Privilege Level
#define NVIC_FPCC_LSPACT        0x00000001  // Lazy State Preservation Active

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPCA register.
//
//*****************************************************************************
#define NVIC_FPCA_ADDRESS_M     0xFFFFFFF8  // Address
#define NVIC_FPCA_ADDRESS_S     3

//*****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPDSC register.
//
//*****************************************************************************
#define NVIC_FPDSC_AHP          0x04000000  // AHP Bit Default
#define NVIC_FPDSC_DN           0x02000000  // DN Bit Default
#define NVIC_FPDSC_FZ           0x01000000  // FZ Bit Default
#define NVIC_FPDSC_RMODE_M      0x00C00000  // RMODE Bit Default
#define NVIC_FPDSC_RMODE_RN     0x00000000  // Round to Nearest (RN) mode
#define NVIC_FPDSC_RMODE_RP     0x00400000  // Round towards Plus Infinity (RP)
                                            // mode
#define NVIC_FPDSC_RMODE_RM     0x00800000  // Round towards Minus Infinity
                                            // (RM) mode
#define NVIC_FPDSC_RMODE_RZ     0x00C00000  // Round towards Zero (RZ) mode

//*****************************************************************************
//
// The following definitions are deprecated.
//
//*****************************************************************************
#ifndef DEPRECATED
#define SYSCTL_DID0_CLASS_BLIZZARD                                            \
                                0x00050000  // Tiva(TM) C Series TM4C123-class
                                            // microcontrollers
#endif

//*****************************************************************************
//
// Generic Macros
//
//*****************************************************************************
#define TRUE                    1
#define FALSE                   0

#define ENABLE                  1
#define DISABLE                 0

#define SET                     ENABLE
#define CLEAR                   DISABLE

#define GPIO_SET                SET
#define GPIO_CLEAR              CLEAR

#endif // __TM4C123GH6PM_H__

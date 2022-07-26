/*tm4c123GH6PM_gpio_driver.h*/
#include <stdint.h>

#ifndef INC_TM4C123GH6PM_GPIO_DRIVER_H_
#define INC_TM4C123GH6PM_GPIO_DRIVER_H_

/*********************************************************************************
*                        Macros used to config registers
*
**********************************************************************************/

/* 
 * GPIO port definations user can pass on in different APIs function 
 * 
 */
 enum Port
 {
   GPIOA_P = 0, 
   GPIOB_P = 1, 
   GPIOC_P = 2,  
   GPIOD_P = 3, 
   GPIOE_P = 4, 
   GPIOF_P = 5, 
 };

/* 
 * GPIO pin definations user can pass on in different APIs function 
 * 
 */
#define PIN_0              0x00000001         // GPIO pin 0
#define PIN_1              0x00000002         // GPIO pin 1
#define PIN_2              0x00000004         // GPIO pin 2
#define PIN_3              0x00000008         // GPIO pin 3
#define PIN_4              0x00000010         // GPIO pin 4
#define PIN_5              0x00000020         // GPIO pin 5
#define PIN_6              0x00000040         // GPIO pin 6
#define PIN_7              0x00000080         // GPIO pin 7

/* 
 * Interrupt assignments 
 * 
 */
#define INT_GPIOA               0                  // GPIO Port A
#define INT_GPIOB               1                  // GPIO Port B
#define INT_GPIOC               2                  // GPIO Port C
#define INT_GPIOD               3                  // GPIO Port D
#define INT_GPIOE               4                  // GPIO Port E
#define INT_GPIOF               30                 // GPIO Port F

/* 
 * GPIO pin mode
 * Arguments to GPIO_Init function
 */
#define GPIO_IN                 0                  // Sets the GPIO pin as input
#define GPIO_OUT                1                  // Sets the GPIO pin as output
#define GPIO_AFSEL              2                  // Enable alternate hardware function for the pin
#define GPIO_ADCCTL             3                  // Enable pin to be used as ADC trigger
#define GPIO_DMACTL             4                  // Enable pin to be used as uDMA trigger


/* 
 * GPIO possible interrupt mode
 * Arguments in GPIO_InterruptTypeSet
 */
#define GPIO_FALLING_EDGE        0x00000000        // Interrupt on falling edge
#define GPIO_RISING_EDGE         0x00000004        // Interrupt on rising edge
#define GPIO_BOTH_EDGES          0x00000001        // Interrupt on both edge
#define GPIO_LOW_LEVEL           0x00000002        // Interrupt on low level
#define GPIO_HIGH_LEVEL          0x00000006        // Interrupt on high level

/* 
 * GPIO Config pad
 * Arguments in GPIO_PadConfig function
 */
#define GPIO_NO_DR               0                // All drives are disabled
#define GPIO_DR2R                1                // Enable 2mA drive on the pin
#define GPIO_DR4R                2                // Enable 4mA drive on the pin
#define GPIO_DR8R                3                // Enable 8mA drive on the pin

/* 
 * GPIO Config pad
 * Arguments in GPIO_PadConfig function
 */
#define GPIO_ODR                 0x00000009        // Pin is configured as open drain
#define GPIO_PUR                 0x0000000A        // Weak pull-up is enabled on pin
#define GPIO_PDR                 0x0000000C        // Weak pull-down is enables on pin
#define GPIO_DEN                 0x00000008        // Enable digital function for the pin, push-pull
#define GPIO_AMSEL               0x00000000        // Enable analog function for the pin

/*
 * 
 * The following are defines for the bit fields in the GPIO_O_LOCK register.
 */
#define GPIO_LOCK_M              0xFFFFFFFF        // GPIO Lock
#define GPIO_LOCK_UNLOCKED       0x00000000        // The GPIOCR register is unlocked and may be modified
#define GPIO_LOCK_LOCKED         0x00000001        // The GPIOCR register is locked and may not be modified
#define GPIO_LOCK_KEY            0x4C4F434B        // Unlocks the GPIO_CR register

/*
 *Arguments used in GPIO_PinConfig, also can be used by alternate function Configuration
 * The following are defines for the bit fields in the GPIO_PCTL register for
 * port A.
*/
#define GPIO_PCTL_PA7_M         0xF0000000         // PA7 Mask
#define GPIO_PCTL_PA7_I2C1SDA   0x30000000         // I2C1SDA on PA7
#define GPIO_PCTL_PA7_M1PWM3    0x50000000         // M1PWM3 on PA7
#define GPIO_PCTL_PA6_M         0x0F000000         // PA6 Mask
#define GPIO_PCTL_PA6_I2C1SCL   0x03000000         // I2C1SCL on PA6
#define GPIO_PCTL_PA6_M1PWM2    0x05000000         // M1PWM2 on PA6
#define GPIO_PCTL_PA5_M         0x00F00000         // PA5 Mask
#define GPIO_PCTL_PA5_SSI0TX    0x00200000         // SSI0TX on PA5
#define GPIO_PCTL_PA4_M         0x000F0000         // PA4 Mask
#define GPIO_PCTL_PA4_SSI0RX    0x00020000         // SSI0RX on PA4
#define GPIO_PCTL_PA3_M         0x0000F000         // PA3 Mask
#define GPIO_PCTL_PA3_SSI0FSS   0x00002000         // SSI0FSS on PA3
#define GPIO_PCTL_PA2_M         0x00000F00         // PA2 Mask
#define GPIO_PCTL_PA2_SSI0CLK   0x00000200         // SSI0CLK on PA2
#define GPIO_PCTL_PA1_M         0x000000F0         // PA1 Mask
#define GPIO_PCTL_PA1_U0TX      0x00000010         // U0TX on PA1
#define GPIO_PCTL_PA1_CAN1TX    0x00000080         // CAN1TX on PA1
#define GPIO_PCTL_PA0_M         0x0000000F         // PA0 Mask
#define GPIO_PCTL_PA0_U0RX      0x00000001         // U0RX on PA0
#define GPIO_PCTL_PA0_CAN1RX    0x00000008         // CAN1RX on PA0

/*
 *Arguments used in GPIO_PinConfig, also can be used by alternate function Configuration
 * The following are defines for the bit fields in the GPIO_PCTL register for
 * port B.
*/
#define GPIO_PCTL_PB7_M         0xF0000000         // PB7 Mask
#define GPIO_PCTL_PB7_SSI2TX    0x20000000         // SSI2TX on PB7
#define GPIO_PCTL_PB7_M0PWM1    0x40000000         // M0PWM1 on PB7
#define GPIO_PCTL_PB7_T0CCP1    0x70000000         // T0CCP1 on PB7
#define GPIO_PCTL_PB6_M         0x0F000000         // PB6 Mask
#define GPIO_PCTL_PB6_SSI2RX    0x02000000         // SSI2RX on PB6
#define GPIO_PCTL_PB6_M0PWM0    0x04000000         // M0PWM0 on PB6
#define GPIO_PCTL_PB6_T0CCP0    0x07000000         // T0CCP0 on PB6
#define GPIO_PCTL_PB5_M         0x00F00000         // PB5 Mask
#define GPIO_PCTL_PB5_SSI2FSS   0x00200000         // SSI2FSS on PB5
#define GPIO_PCTL_PB5_M0PWM3    0x00400000         // M0PWM3 on PB5
#define GPIO_PCTL_PB5_T1CCP1    0x00700000         // T1CCP1 on PB5
#define GPIO_PCTL_PB5_CAN0TX    0x00800000         // CAN0TX on PB5
#define GPIO_PCTL_PB4_M         0x000F0000         // PB4 Mask
#define GPIO_PCTL_PB4_SSI2CLK   0x00020000         // SSI2CLK on PB4
#define GPIO_PCTL_PB4_M0PWM2    0x00040000         // M0PWM2 on PB4
#define GPIO_PCTL_PB4_T1CCP0    0x00070000         // T1CCP0 on PB4
#define GPIO_PCTL_PB4_CAN0RX    0x00080000         // CAN0RX on PB4
#define GPIO_PCTL_PB3_M         0x0000F000         // PB3 Mask
#define GPIO_PCTL_PB3_I2C0SDA   0x00003000         // I2C0SDA on PB3
#define GPIO_PCTL_PB3_T3CCP1    0x00007000         // T3CCP1 on PB3
#define GPIO_PCTL_PB2_M         0x00000F00         // PB2 Mask
#define GPIO_PCTL_PB2_I2C0SCL   0x00000300         // I2C0SCL on PB2
#define GPIO_PCTL_PB2_T3CCP0    0x00000700         // T3CCP0 on PB2
#define GPIO_PCTL_PB1_M         0x000000F0         // PB1 Mask
#define GPIO_PCTL_PB1_USB0VBUS  0x00000000         // USB0VBUS on PB1
#define GPIO_PCTL_PB1_U1TX      0x00000010         // U1TX on PB1
#define GPIO_PCTL_PB1_T2CCP1    0x00000070         // T2CCP1 on PB1
#define GPIO_PCTL_PB0_M         0x0000000F         // PB0 Mask
#define GPIO_PCTL_PB0_USB0ID    0x00000000         // USB0ID on PB0
#define GPIO_PCTL_PB0_U1RX      0x00000001         // U1RX on PB0
#define GPIO_PCTL_PB0_T2CCP0    0x00000007         // T2CCP0 on PB0

/*
 *Arguments used in GPIO_PinConfig, also can be used by alternate function Configuration
 * The following are defines for the bit fields in the GPIO_PCTL register for
 * port C.
*/
#define GPIO_PCTL_PC7_M         0xF0000000         // PC7 Mask
#define GPIO_PCTL_PC7_U3TX      0x10000000         // U3TX on PC7
#define GPIO_PCTL_PC7_WT1CCP1   0x70000000         // WT1CCP1 on PC7
#define GPIO_PCTL_PC7_USB0PFLT  0x80000000         // USB0PFLT on PC7
#define GPIO_PCTL_PC6_M         0x0F000000         // PC6 Mask
#define GPIO_PCTL_PC6_U3RX      0x01000000         // U3RX on PC6
#define GPIO_PCTL_PC6_PHB1      0x06000000         // PHB1 on PC6
#define GPIO_PCTL_PC6_WT1CCP0   0x07000000         // WT1CCP0 on PC6
#define GPIO_PCTL_PC6_USB0EPEN  0x08000000         // USB0EPEN on PC6
#define GPIO_PCTL_PC5_M         0x00F00000         // PC5 Mask
#define GPIO_PCTL_PC5_U4TX      0x00100000         // U4TX on PC5
#define GPIO_PCTL_PC5_U1TX      0x00200000         // U1TX on PC5
#define GPIO_PCTL_PC5_M0PWM7    0x00400000         // M0PWM7 on PC5
#define GPIO_PCTL_PC5_PHA1      0x00600000         // PHA1 on PC5
#define GPIO_PCTL_PC5_WT0CCP1   0x00700000         // WT0CCP1 on PC5
#define GPIO_PCTL_PC5_U1CTS     0x00800000         // U1CTS on PC5
#define GPIO_PCTL_PC4_M         0x000F0000         // PC4 Mask
#define GPIO_PCTL_PC4_U4RX      0x00010000         // U4RX on PC4
#define GPIO_PCTL_PC4_U1RX      0x00020000         // U1RX on PC4
#define GPIO_PCTL_PC4_M0PWM6    0x00040000         // M0PWM6 on PC4
#define GPIO_PCTL_PC4_IDX1      0x00060000         // IDX1 on PC4
#define GPIO_PCTL_PC4_WT0CCP0   0x00070000         // WT0CCP0 on PC4
#define GPIO_PCTL_PC4_U1RTS     0x00080000         // U1RTS on PC4
#define GPIO_PCTL_PC3_M         0x0000F000         // PC3 Mask
#define GPIO_PCTL_PC3_TDO       0x00001000         // TDO on PC3
#define GPIO_PCTL_PC3_T5CCP1    0x00007000         // T5CCP1 on PC3
#define GPIO_PCTL_PC2_M         0x00000F00         // PC2 Mask
#define GPIO_PCTL_PC2_TDI       0x00000100         // TDI on PC2
#define GPIO_PCTL_PC2_T5CCP0    0x00000700         // T5CCP0 on PC2
#define GPIO_PCTL_PC1_M         0x000000F0         // PC1 Mask
#define GPIO_PCTL_PC1_TMS       0x00000010         // TMS on PC1
#define GPIO_PCTL_PC1_T4CCP1    0x00000070         // T4CCP1 on PC1
#define GPIO_PCTL_PC0_M         0x0000000F         // PC0 Mask
#define GPIO_PCTL_PC0_TCK       0x00000001         // TCK on PC0
#define GPIO_PCTL_PC0_T4CCP0    0x00000007         // T4CCP0 on PC0

/*
 *Arguments used in GPIO_PinConfig, also can be used by alternate function Configuration
 * The following are defines for the bit fields in the GPIO_PCTL register for
 * port D.
*/
#define GPIO_PCTL_PD7_M         0xF0000000         // PD7 Mask
#define GPIO_PCTL_PD7_U2TX      0x10000000         // U2TX on PD7
#define GPIO_PCTL_PD7_PHB0      0x60000000         // PHB0 on PD7
#define GPIO_PCTL_PD7_WT5CCP1   0x70000000         // WT5CCP1 on PD7
#define GPIO_PCTL_PD7_NMI       0x80000000         // NMI on PD7
#define GPIO_PCTL_PD6_M         0x0F000000         // PD6 Mask
#define GPIO_PCTL_PD6_U2RX      0x01000000         // U2RX on PD6
#define GPIO_PCTL_PD6_M0FAULT0  0x04000000         // M0FAULT0 on PD6
#define GPIO_PCTL_PD6_PHA0      0x06000000         // PHA0 on PD6
#define GPIO_PCTL_PD6_WT5CCP0   0x07000000         // WT5CCP0 on PD6
#define GPIO_PCTL_PD5_M         0x00F00000         // PD5 Mask
#define GPIO_PCTL_PD5_USB0DP    0x00000000         // USB0DP on PD5
#define GPIO_PCTL_PD5_U6TX      0x00100000         // U6TX on PD5
#define GPIO_PCTL_PD5_WT4CCP1   0x00700000         // WT4CCP1 on PD5
#define GPIO_PCTL_PD4_M         0x000F0000         // PD4 Mask
#define GPIO_PCTL_PD4_USB0DM    0x00000000         // USB0DM on PD4
#define GPIO_PCTL_PD4_U6RX      0x00010000         // U6RX on PD4
#define GPIO_PCTL_PD4_WT4CCP0   0x00070000         // WT4CCP0 on PD4
#define GPIO_PCTL_PD3_M         0x0000F000         // PD3 Mask
#define GPIO_PCTL_PD3_AIN4      0x00000000         // AIN4 on PD3
#define GPIO_PCTL_PD3_SSI3TX    0x00001000         // SSI3TX on PD3
#define GPIO_PCTL_PD3_SSI1TX    0x00002000         // SSI1TX on PD3
#define GPIO_PCTL_PD3_IDX0      0x00006000         // IDX0 on PD3
#define GPIO_PCTL_PD3_WT3CCP1   0x00007000         // WT3CCP1 on PD3
#define GPIO_PCTL_PD3_USB0PFLT  0x00008000         // USB0PFLT on PD3
#define GPIO_PCTL_PD2_M         0x00000F00         // PD2 Mask
#define GPIO_PCTL_PD2_AIN5      0x00000000         // AIN5 on PD2
#define GPIO_PCTL_PD2_SSI3RX    0x00000100         // SSI3RX on PD2
#define GPIO_PCTL_PD2_SSI1RX    0x00000200         // SSI1RX on PD2
#define GPIO_PCTL_PD2_M0FAULT0  0x00000400         // M0FAULT0 on PD2
#define GPIO_PCTL_PD2_WT3CCP0   0x00000700         // WT3CCP0 on PD2
#define GPIO_PCTL_PD2_USB0EPEN  0x00000800         // USB0EPEN on PD2
#define GPIO_PCTL_PD1_M         0x000000F0         // PD1 Mask
#define GPIO_PCTL_PD1_AIN6      0x00000000         // AIN6 on PD1
#define GPIO_PCTL_PD1_SSI3FSS   0x00000010         // SSI3FSS on PD1
#define GPIO_PCTL_PD1_SSI1FSS   0x00000020         // SSI1FSS on PD1
#define GPIO_PCTL_PD1_I2C3SDA   0x00000030         // I2C3SDA on PD1
#define GPIO_PCTL_PD1_M0PWM7    0x00000040         // M0PWM7 on PD1
#define GPIO_PCTL_PD1_M1PWM1    0x00000050         // M1PWM1 on PD1
#define GPIO_PCTL_PD1_WT2CCP1   0x00000070         // WT2CCP1 on PD1
#define GPIO_PCTL_PD0_M         0x0000000F         // PD0 Mask
#define GPIO_PCTL_PD0_AIN7      0x00000000         // AIN7 on PD0
#define GPIO_PCTL_PD0_SSI3CLK   0x00000001         // SSI3CLK on PD0
#define GPIO_PCTL_PD0_SSI1CLK   0x00000002         // SSI1CLK on PD0
#define GPIO_PCTL_PD0_I2C3SCL   0x00000003         // I2C3SCL on PD0
#define GPIO_PCTL_PD0_M0PWM6    0x00000004         // M0PWM6 on PD0
#define GPIO_PCTL_PD0_M1PWM0    0x00000005         // M1PWM0 on PD0
#define GPIO_PCTL_PD0_WT2CCP0   0x00000007         // WT2CCP0 on PD0

/*
 *Arguments used in GPIO_PinConfig, also can be used by alternate function Configuration
 * The following are defines for the bit fields in the GPIO_PCTL register for
 * port E.
*/
#define GPIO_PCTL_PE5_M         0x00F00000         // PE5 Mask
#define GPIO_PCTL_PE5_AIN8      0x00000000         // AIN8 on PE5
#define GPIO_PCTL_PE5_U5TX      0x00100000         // U5TX on PE5
#define GPIO_PCTL_PE5_I2C2SDA   0x00300000         // I2C2SDA on PE5
#define GPIO_PCTL_PE5_M0PWM5    0x00400000         // M0PWM5 on PE5
#define GPIO_PCTL_PE5_M1PWM3    0x00500000         // M1PWM3 on PE5
#define GPIO_PCTL_PE5_CAN0TX    0x00800000         // CAN0TX on PE5
#define GPIO_PCTL_PE4_M         0x000F0000         // PE4 Mask
#define GPIO_PCTL_PE4_AIN9      0x00000000         // AIN9 on PE4
#define GPIO_PCTL_PE4_U5RX      0x00010000         // U5RX on PE4
#define GPIO_PCTL_PE4_I2C2SCL   0x00030000         // I2C2SCL on PE4
#define GPIO_PCTL_PE4_M0PWM4    0x00040000         // M0PWM4 on PE4
#define GPIO_PCTL_PE4_M1PWM2    0x00050000         // M1PWM2 on PE4
#define GPIO_PCTL_PE4_CAN0RX    0x00080000         // CAN0RX on PE4
#define GPIO_PCTL_PE3_M         0x0000F000         // PE3 Mask
#define GPIO_PCTL_PE3_AIN0      0x00000000         // AIN0 on PE3
#define GPIO_PCTL_PE2_M         0x00000F00         // PE2 Mask
#define GPIO_PCTL_PE2_AIN1      0x00000000         // AIN1 on PE2
#define GPIO_PCTL_PE1_M         0x000000F0         // PE1 Mask
#define GPIO_PCTL_PE1_AIN2      0x00000000         // AIN2 on PE1
#define GPIO_PCTL_PE1_U7TX      0x00000010         // U7TX on PE1
#define GPIO_PCTL_PE0_M         0x0000000F         // PE0 Mask
#define GPIO_PCTL_PE0_AIN3      0x00000000         // AIN3 on PE0
#define GPIO_PCTL_PE0_U7RX      0x00000001         // U7RX on PE0

/*
 *Arguments used in GPIO_PinConfig, also can be used by alternate function Configuration
 * The following are defines for the bit fields in the GPIO_PCTL register for
 * port F.
*/
#define GPIO_PCTL_PF4_M         0x000F0000         // PF4 Mask
#define GPIO_PCTL_PF4_M1FAULT0  0x00050000         // M1FAULT0 on PF4
#define GPIO_PCTL_PF4_IDX0      0x00060000         // IDX0 on PF4
#define GPIO_PCTL_PF4_T2CCP0    0x00070000         // T2CCP0 on PF4
#define GPIO_PCTL_PF4_USB0EPEN  0x00080000         // USB0EPEN on PF4
#define GPIO_PCTL_PF3_M         0x0000F000         // PF3 Mask
#define GPIO_PCTL_PF3_SSI1FSS   0x00002000         // SSI1FSS on PF3
#define GPIO_PCTL_PF3_CAN0TX    0x00003000         // CAN0TX on PF3
#define GPIO_PCTL_PF3_M1PWM7    0x00005000         // M1PWM7 on PF3
#define GPIO_PCTL_PF3_T1CCP1    0x00007000         // T1CCP1 on PF3
#define GPIO_PCTL_PF3_TRCLK     0x0000E000         // TRCLK on PF3
#define GPIO_PCTL_PF2_M         0x00000F00         // PF2 Mask
#define GPIO_PCTL_PF2_SSI1CLK   0x00000200         // SSI1CLK on PF2
#define GPIO_PCTL_PF2_M0FAULT0  0x00000400         // M0FAULT0 on PF2
#define GPIO_PCTL_PF2_M1PWM6    0x00000500         // M1PWM6 on PF2
#define GPIO_PCTL_PF2_T1CCP0    0x00000700         // T1CCP0 on PF2
#define GPIO_PCTL_PF2_TRD0      0x00000E00         // TRD0 on PF2
#define GPIO_PCTL_PF1_M         0x000000F0         // PF1 Mask
#define GPIO_PCTL_PF1_U1CTS     0x00000010         // U1CTS on PF1
#define GPIO_PCTL_PF1_SSI1TX    0x00000020         // SSI1TX on PF1
#define GPIO_PCTL_PF1_M1PWM5    0x00000050         // M1PWM5 on PF1
#define GPIO_PCTL_PF1_PHB0      0x00000060         // PHB0 on PF1
#define GPIO_PCTL_PF1_T0CCP1    0x00000070         // T0CCP1 on PF1
#define GPIO_PCTL_PF1_C1O       0x00000090         // C1O on PF1
#define GPIO_PCTL_PF1_TRD1      0x000000E0         // TRD1 on PF1
#define GPIO_PCTL_PF0_M         0x0000000F         // PF0 Mask
#define GPIO_PCTL_PF0_U1RTS     0x00000001         // U1RTS on PF0
#define GPIO_PCTL_PF0_SSI1RX    0x00000002         // SSI1RX on PF0
#define GPIO_PCTL_PF0_CAN0RX    0x00000003         // CAN0RX on PF0
#define GPIO_PCTL_PF0_M1PWM4    0x00000005         // M1PWM4 on PF0
#define GPIO_PCTL_PF0_PHA0      0x00000006         // PHA0 on PF0
#define GPIO_PCTL_PF0_T0CCP0    0x00000007         // T0CCP0 on PF0
#define GPIO_PCTL_PF0_NMI       0x00000008         // NMI on PF0
#define GPIO_PCTL_PF0_C0O       0x00000009         // C0O on PF0

/*********************************************************************************
*                        API supported by this driver
*
**********************************************************************************/

/*
 * Peripheral setup
 *
 */
void GPIO_Init(uint8_t GPIOx, uint8_t Pinx, uint8_t PinIO);
void GPIO_Deinit(uint8_t GPIOx, uint8_t Pinx);
void GPIO_EnableAHB(uint8_t GPIOx);
void GPIO_DisableAHB(uint8_t GPIOx);

/*
 * Configure port and pin
 *
 */
void GPIO_PadConfig(uint8_t GPIOx, uint8_t Pinx, uint8_t Strength, uint8_t PinType);
void GPIO_PinConfig(uint8_t GPIOx, uint32_t GPIO_PCTL);
void GPIO_EnableSLR(uint8_t GPIOx, uint8_t Pinx);
void GPIO_DisableSLR(uint8_t GPIOx, uint8_t Pinx);



/*
 * Data read and write
 *
 */

uint8_t GPIO_ReadInputPort(uint8_t GPIOx);
uint8_t GPIO_ReadInputPin(uint8_t GPIOx, uint8_t Pinx);
void GPIO_WriteOutputPort(uint8_t GPIOx, uint8_t Value);
void GPIO_WriteOutputPin(uint8_t GPIOx, uint8_t Pinx, uint8_t Value);
void GPIO_ToggleOutputPin(uint8_t GPIOx, uint8_t Pinx);

/*
 * Locks and  unlocks pin
 *
 */

void GPIO_Lock(uint8_t GPIOx, uint8_t Pinx);
void GPIO_Unlock(uint8_t GPIOx, uint8_t Pinx);

/*
 * IRQ Configuration and ISR handling
 *
 */
void GPIO_InterruptTypeSet(uint8_t GPIOx, uint8_t Pinx, uint8_t IntType);
void GPIO_EnableInterrupt(uint8_t GPIOx, uint8_t Pinx);
void GPIO_DisableInterrupt(uint8_t GPIOx, uint8_t Pinx);
uint8_t GPIO_GetInterruptStatus(uint8_t GPIOx, uint8_t Pinx);
void GPIO_ClearInterrupt(uint8_t GPIOx, uint8_t Pinx);
void GPIO_IRQConfig(uint8_t IRQn, uint8_t Ctrl);
void GPIO_IRQPriorityConfig(uint8_t IRQn, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t GPIOx, uint8_t Pinx);

/*
 * Alternate function Configuration
 *
 */
 void GPIO_SSIType(uint8_t GPIOx, uint8_t Pinx, uint32_t GPIO_PCTL);
 void GPIO_UARTType(uint8_t GPIOx, uint8_t Pinx, uint32_t GPIO_PCTL);
 void GPIO_I2CTypeSDA(uint8_t GPIOx, uint8_t Pinx, uint32_t GPIO_PCTL);
 void GPIO_I2CTypeSCL(uint8_t GPIOx, uint8_t Pinx, uint32_t GPIO_PCTL);
 
/*
 * Helping macros
 *
 */
 #define ENABLE  1
 #define DISABLE 0

#endif  // __TM4C123GH6PM_GPIO_DRIVER_H__

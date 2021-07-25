/*tm4c123GH6PM_spi_driver.h*/
#include <stdint.h>

#ifndef INC_TM4C123GH6PM_SPI_DRIVER_H_
#define INC_TM4C123GH6PM_SPI_DRIVER_H_


/*********************************************************************************
*                        Macros used to config registers
*
**********************************************************************************/

 enum Module
 {
   SSI0_P = 0, 
   SSI1_P = 1,
   SSI2_P = 2,
   SSI3_P = 3
 };

/* 
 * SSI module run mode clock gating control
 *  Arguments to SSI_EnableClk and SSI_DisableClk functions
 */
#define SYSCTL_RCGCSSI_R3       0x00000008  // SSI Module 3 Run Mode Clock Gating Control
#define SYSCTL_RCGCSSI_R2       0x00000004  // SSI Module 2 Run Mode Clock Gating Control
#define SYSCTL_RCGCSSI_R1       0x00000002  // SSI Module 1 Run Mode Clock Gating Control
#define SYSCTL_RCGCSSI_R0       0x00000001  // SSI Module 0 Run Mode Clock Gating Control

/* 
 * Confingure SSI module
 * Arguments to SSI_ConfigModule function
 */
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_MODE_0          0x00000000  // polarity 0, phase 0
#define SSI_CR1_MODE_1          0x00000080  // polarity 0, phase 1
#define SSI_CR2_MODE_2          0x00000040  // polarity 1, phase 0
#define SSI_CR3_MODE_3          0x000000C0  // polarity 1, phase 1

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

/* 
 * Confingure SSI module
 * Arguments to SET_SSIMode function
 */
#define SSI_CR1_EOT             0x00000010  // End of Transmission
#define SSI_CR1_M               0x00000000  // SSI Master Select
#define SSI_CR1_S               0x00000004  // SSI Slave Select
#define SSI_CR1_SD              0x0000000C  // Slave mode output disabled
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port Enable
#define SSI_CR1_LBM             0x00000001  // SSI Loopback Mode

/* 
 * Reset SSI modulese
 * Arguments to Reset function
 */
#define SYSCTL_SRSSI_R3         0x00000008  // SSI Module 3 Software Reset
#define SYSCTL_SRSSI_R2         0x00000004  // SSI Module 2 Software Reset
#define SYSCTL_SRSSI_R1         0x00000002  // SSI Module 1 Software Reset
#define SYSCTL_SRSSI_R0         0x00000001  // SSI Module 0 Software Reset

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
#define SSI_IM_RTIM             0x00000002  // SSI Receive Time-Out Interrupt Mask
#define SSI_IM_RORIM            0x00000001  // SSI Receive Overrun Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
//*****************************************************************************
#define SSI_RIS_TXRIS           0x00000008  // SSI Transmit FIFO Raw Interrupt Status
#define SSI_RIS_RXRIS           0x00000004  // SSI Receive FIFO Raw Interrupt Status
#define SSI_RIS_RTRIS           0x00000002  // SSI Receive Time-Out Raw Interrupt Status
#define SSI_RIS_RORRIS          0x00000001  // SSI Receive Overrun Raw Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
//*****************************************************************************
#define SSI_MIS_TXMIS           0x00000008  // SSI Transmit FIFO Masked Interrupt Status
#define SSI_MIS_RXMIS           0x00000004  // SSI Receive FIFO Masked Interrupt Status
#define SSI_MIS_RTMIS           0x00000002  // SSI Receive Time-Out Masked Interrupt Status
#define SSI_MIS_RORMIS          0x00000001  // SSI Receive Overrun Masked Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
//*****************************************************************************
#define SSI_ICR_RTIC            0x00000002  // SSI Receive Time-Out Interrupt Clear
#define SSI_ICR_RORIC           0x00000001  // SSI Receive Overrun Interrupt  Clear

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
#define SSI_CC_CS_SYSPLL        0x00000000  // System clock (based on clock source and divisor factor)
#define SSI_CC_CS_PIOSC         0x00000005  // PIOSC

/*********************************************************************************
*                        API supported by this driver
*
**********************************************************************************/

/*
 * Peripheral setup
 *
 */
void SSI_EnableClk(uint8_t SYSCTL_RCGCSSI_MODULE);
void SSI_DisableClk(uint8_t SYSCTL_RCGCSSI_MODULE);
void SSI_EnableModule(uint8_t SSIx);
void SSI_DisableModule(uint8_t SSIx);
void SSI_Reset(uint8_t SYSCTL_SRSSI);

/*
 * Peripheral configure
 *
 */
 void SET_SSIMode(uint8_t SSIx, uint32_t Mode);
 void SSI_ConfigClk(uint8_t SSIx, uint32_t SSIClk, uint32_t BitRate);
 void SSI_ConfigModule(uint8_t SSIx, uint32_t PhasePolMode, uint32_t ProtocolMode, uint32_t DSS);

 /*
 * Data read and write
 *
 */
 void SPI_SendData(uint8_t SSIx, uint16_t Data);
 void SPI_ReceiveData(uint8_t SSIx, uint16_t *Data);
 void EnableLoopbackMode(uint8_t SSIx);
 void DisableLoopbackMode(uint8_t SSIx);

 /*
 * Other
 *
 */
 uint8_t SSI_Busy(uint8_t SSIx);

#endif  // __TM4C123GH6PM_GPIO_DRIVER_H__
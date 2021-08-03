/*tm4c123GH6PM_uart_driver.h*/
#include <stdint.h>

#ifndef INC_TM4C123GH6PM_UART_DRIVER_H_
#define INC_TM4C123GH6PM_UART_DRIVER_H_


/*********************************************************************************
*                        Macros used to config registers
*
**********************************************************************************/
enum UART_Module
 {
   UART0_P = 0, 
   UART1_P = 1,
   UART2_P = 2,
   UART3_P = 3,
   UART4_P = 4,
   UART5_P = 5,
   UART6_P = 6,
   UART7_P = 7
   
 };

/* 
 * UART module run mode clock gating control
 * Arguments to UART_EnableClk and UART_DisableClk functions
 */
#define SYSCTL_RCGCUART_R7      0x00000080  // UART Module 7 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R6      0x00000040  // UART Module 6 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R5      0x00000020  // UART Module 5 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R4      0x00000010  // UART Module 4 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R3      0x00000008  // UART Module 3 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R2      0x00000004  // UART Module 2 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R1      0x00000002  // UART Module 1 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R0      0x00000001  // UART Module 0 Run Mode Clock Gating Control

/* 
 * Configure UART module
 * Arguments to
 */
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

/* 
 * Configure UART module
 * Arguments to UART_ConfigModule
 */
#define UART_LCRH_WLEN_M        0x00000060  // UART Word Length
#define UART_LCRH_WLEN_5        0x00000000  // 5 bits (default)
#define UART_LCRH_WLEN_6        0x00000020  // 6 bits
#define UART_LCRH_WLEN_7        0x00000040  // 7 bits
#define UART_LCRH_WLEN_8        0x00000060  // 8 bits

#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs

#define UART_LCRH_STP1          0x00000000  // UART One Stop Bits Select
#define UART_LCRH_STP2          0x00000008  // UART Two Stop Bits Select

#define UART_LCRH_NONE          0x00000000  // UART No parity
#define UART_LCRH_EVEN          0x00000006  // UART Even parity
#define UART_LCRH_ODD           0x00000002  // UART Odd parity
#define UART_LCRH_ONE           0x00000082  // UART Parity bit is one
#define UART_LCRH_ZERO          0x00000086  // UART Parity bit is zero

#define UART_LCRH_BRK           0x00000001  // UART Send Break

/* 
 * Reset UART Moudles
 * Arguments to UART_RESET
 */
#define SYSCTL_SRUART_R7        0x00000080  // UART Module 7 Software Reset
#define SYSCTL_SRUART_R6        0x00000040  // UART Module 6 Software Reset
#define SYSCTL_SRUART_R5        0x00000020  // UART Module 5 Software Reset
#define SYSCTL_SRUART_R4        0x00000010  // UART Module 4 Software Reset
#define SYSCTL_SRUART_R3        0x00000008  // UART Module 3 Software Reset
#define SYSCTL_SRUART_R2        0x00000004  // UART Module 2 Software Reset
#define SYSCTL_SRUART_R1        0x00000002  // UART Module 1 Software Reset
#define SYSCTL_SRUART_R0        0x00000001  // UART Module 0 Software Reset

/* 
 * UART Flags
 * To check status
 */
#define UART_FR_TXFE            0x00000080  // UART Transmit FIFO Empty
#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_FR_BUSY            0x00000008  // UART Busy
#define UART_FR_CTS             0x00000001  // Clear To Send


/*
 * Peripheral setup
 *
 */
void UART_EnableClk(uint8_t SYSCTL_RCGCUART_MODULE);
void UART_DisableClk(uint8_t SYSCTL_RCGCUART_MODULE);
void UART_EnableModule(uint8_t UARTx);
void UART_DisableModule(uint8_t UARTx);
void UART_Reset(uint8_t SYSCTL_SRUART);
void UART_FIFOEnable(uint8_t UARTx);
void UART_FIFODisable(uint8_t UARTx);

/*
 * Peripheral configure
 *
 */
 void UART_ConfigModule(uint8_t UARTx, uint32_t Clk, uint32_t Baud, uint32_t Parity, uint32_t DataLength, uint32_t StopBits);

  /*
 * Data read and write
 *
 */
 void UART_WriteChar(uint8_t UARTx, unsigned char data);
 void UART_WriteString(uint8_t UARTx, unsigned char buffer[]);
 uint8_t UART_ReadChar(uint8_t UARTx);
#endif  // __TM4C123GH6PM_GPIO_DRIVER_H__
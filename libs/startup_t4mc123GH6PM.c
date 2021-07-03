//Startup code for TM4C MCU

#include <stdint.h>


//SRAM grows from higher address to lower. Define space for stack pointer

#define SRAM_START 0x20000000U
#define SRAM_SIZE (32U * 1024U) //32KB
#define SRAM_END ((SRAM_START) + (SRAM_SIZE))

#define STACK_START SRAM_END

/* 
 The following constructers are created by the linker, indicating the starts and the ends of the data and bss segments whicg resides in the memory
 */
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _la_data;
extern uint32_t _edata;

extern uint32_t _sbss;
extern uint32_t _ebss;

//Prototype of main

int main(void);

void __libc_init_array(void);

//Function prototypes of TM4C system exception and IRQ handlers

void Reset_Handler(void);

void NMI_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortA_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortB_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortC_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortD_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortE_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMFault_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen0_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen1_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen2_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq0_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq1_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq2_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void ADCSeq3_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Watchdog_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer0A_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer0B_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer1A_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer1B_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer2A_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer2B_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Comp0_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void Comp1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void Comp2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void SysCtrl_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void FlashCtrl_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortF_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortG_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortH_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer3A_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer3B_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void Hibernate_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void USB0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void PWMGen3_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void uDMAST_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void uDMAError_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1Seq3_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortJ_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortK_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortL_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void SSI3_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void UART3_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART6_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UART7_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer4A_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer4B_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer5A_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void Timer5B_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer0A_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer0B_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer1A_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer1B_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer2A_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer2B_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer3A_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer3B_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer4A_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer4B_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer5A_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void WideTimer5B_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C4_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C5_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortM_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortN_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void QEI2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP1_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP2_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP3_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP4_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP5_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP6_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortP7_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ1_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ2_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ3_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ4_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ5_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ6_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortQ7_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortR_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIOPortS_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Gen3_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1Fault_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));


/* The vector table.  Note that the proper constructs must be placed on this to
   ensure that it ends up at physical address 0x0000.0000.*/


uint32_t const vector[]__attribute__ ((section(".isr_vector"))) =
{
  STACK_START,                                // The initial stack pointer
  (uint32_t)Reset_Handler,                    // The reset handler
  (uint32_t)NMI_Handler,                      // The NMI handler
  (uint32_t)HardFault_Handler,                // The hard fault handler
  (uint32_t)MemManage_Handler,                // The MPU fault handler
  (uint32_t)BusFault_Handler,                 // The bus fault handler
  (uint32_t)UsageFault_Handler,               // The usage fault handler
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
  (uint32_t)SVC_Handler,                      // SVCall handler
  (uint32_t)DebugMon_Handler,                 // Debug monitor handler
    0,                                        // Reserved
  (uint32_t)PendSV_Handler,                   // The PendSV handler
  (uint32_t)SysTick_Handler,                  // The SysTick handler
  (uint32_t)GPIOPortA_IRQHandler,             // GPIO Port A
  (uint32_t)GPIOPortB_IRQHandler,             // GPIO Port B
  (uint32_t)GPIOPortC_IRQHandler,             // GPIO Port C
  (uint32_t)GPIOPortD_IRQHandler,             // GPIO Port D
  (uint32_t)GPIOPortE_IRQHandler,             // GPIO Port E
  (uint32_t)UART0_IRQHandler,                 // UART0 Rx and Tx
  (uint32_t)UART1_IRQHandler,                 // UART1 Rx and Tx
  (uint32_t)SSI0_IRQHandler,                  // SSI0 Rx and Tx
  (uint32_t)I2C0_IRQHandler,                  // I2C0 Master and Slave
  (uint32_t)PWMFault_IRQHandler,              // PWM Fault
  (uint32_t)PWMGen0_IRQHandler,               // PWM Generator 0
  (uint32_t)PWMGen1_IRQHandler,               // PWM Generator 1
  (uint32_t)PWMGen2_IRQHandler,               // PWM Generator 2
  (uint32_t)QEI0_IRQHandler,                  // Quadrature Encoder 0
  (uint32_t)ADCSeq0_IRQHandler,               // ADC Sequence 0
  (uint32_t)ADCSeq1_IRQHandler,               // ADC Sequence 1
  (uint32_t)ADCSeq2_IRQHandler,               // ADC Sequence 2
  (uint32_t)ADCSeq3_IRQHandler,               // ADC Sequence 3
  (uint32_t)Watchdog_IRQHandler,              // Watchdog timer
  (uint32_t)Timer0A_IRQHandler,               // Timer 0 subtimer A
  (uint32_t)Timer0B_IRQHandler,               // Timer 0 subtimer B
  (uint32_t)Timer1A_IRQHandler,               // Timer 1 subtimer A
  (uint32_t)Timer1B_IRQHandler,               // Timer 1 subtimer B
  (uint32_t)Timer2A_IRQHandler,               // Timer 2 subtimer A
  (uint32_t)Timer2B_IRQHandler,               // Timer 2 subtimer B
  (uint32_t)Comp0_IRQHandler,                 // Analog Comparator 0
  (uint32_t)Comp1_IRQHandler,                 // Analog Comparator 1
  (uint32_t)Comp2_IRQHandler,                 // Analog Comparator 2
  (uint32_t)SysCtrl_IRQHandler,               // System Control (PLL, OSC, BO)
  (uint32_t)FlashCtrl_IRQHandler,             // FLASH Control
  (uint32_t)GPIOPortF_IRQHandler,             // GPIO Port F
  (uint32_t)GPIOPortG_IRQHandler,             // GPIO Port G
  (uint32_t)GPIOPortH_IRQHandler,             // GPIO Port H
  (uint32_t)UART2_IRQHandler,                 // UART2 Rx and Tx
  (uint32_t)SSI1_IRQHandler,                  // SSI1 Rx and Tx
  (uint32_t)Timer3A_IRQHandler,               // Timer 3 subtimer A
  (uint32_t)Timer3B_IRQHandler,               // Timer 3 subtimer B
  (uint32_t)I2C1_IRQHandler,                  // I2C1 Master and Slave
  (uint32_t)QEI1_IRQHandler,                  // Quadrature Encoder 1
  (uint32_t)CAN0_IRQHandler,                  // CAN0
  (uint32_t)CAN1_IRQHandler,                  // CAN1
  (uint32_t)CAN2_IRQHandler,                  // Can2
    0,                                        // Reserved
  (uint32_t)Hibernate_IRQHandler,             // Hibernate
  (uint32_t)USB0_IRQHandler,                  // USB0
  (uint32_t)PWMGen3_IRQHandler,               // PWM Generator 3
  (uint32_t)uDMAST_IRQHandler,                // uDMA Software Transfer
  (uint32_t)uDMAError_IRQHandler,             // uDMA Error
  (uint32_t)ADC1Seq0_IRQHandler,              // ADC1 Sequence 0
  (uint32_t)ADC1Seq1_IRQHandler,              // ADC1 Sequence 1
  (uint32_t)ADC1Seq2_IRQHandler,              // ADC1 Sequence 2
  (uint32_t)ADC1Seq3_IRQHandler,              // ADC1 Sequence 3
    0,                                        // Reserved
    0,                                        // Reserved
  (uint32_t)GPIOPortJ_IRQHandler,             // GPIO Port J
  (uint32_t)GPIOPortK_IRQHandler,             // GPIO Port K
  (uint32_t)GPIOPortL_IRQHandler,             // GPIO Port L
  (uint32_t)SSI2_IRQHandler,                  // SSI2 Rx and Tx
  (uint32_t)SSI3_IRQHandler,                  // SSI3 Rx and Tx
  (uint32_t)UART3_IRQHandler,                 // UART3 Rx and Tx
  (uint32_t)UART4_IRQHandler,                 // UART4 Rx and Tx
  (uint32_t)UART5_IRQHandler,                 // UART5 Rx and Tx
  (uint32_t)UART6_IRQHandler,                 // UART6 Rx and Tx
  (uint32_t) UART7_IRQHandler,                // UART7 Rx and Tx
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
  (uint32_t)I2C2_IRQHandler,                  // I2C2 Master and Slave
  (uint32_t)I2C3_IRQHandler,                  // I2C3 Master and Slave
  (uint32_t)Timer4A_IRQHandler,               // Timer 4 subtimer A
  (uint32_t)Timer4B_IRQHandler,               // Timer 4 subtimer B
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
    0,                                        // Reserved
   (uint32_t)Timer5A_IRQHandler,              // Timer 5 subtimer A
   (uint32_t)Timer5B_IRQHandler,              // Timer 5 subtimer B
   (uint32_t)WideTimer0A_IRQHandler,          // Wide Timer 0 subtimer A
   (uint32_t)WideTimer0B_IRQHandler,          // Wide Timer 0 subtimer B
   (uint32_t)WideTimer1A_IRQHandler,          // Wide Timer 1 subtimer A
   (uint32_t)WideTimer1B_IRQHandler,          // Wide Timer 1 subtimer B
   (uint32_t)WideTimer2A_IRQHandler,          // Wide Timer 2 subtimer A
   (uint32_t)WideTimer2B_IRQHandler,          // Wide Timer 2 subtimer B
   (uint32_t)WideTimer3A_IRQHandler,          // Wide Timer 3 subtimer A
   (uint32_t)WideTimer3B_IRQHandler,          // Wide Timer 3 subtimer B
   (uint32_t)WideTimer4A_IRQHandler,          // Wide Timer 4 subtimer A
   (uint32_t)WideTimer4B_IRQHandler,          // Wide Timer 4 subtimer B
   (uint32_t)WideTimer5A_IRQHandler,          // Wide Timer 5 subtimer A
   (uint32_t)WideTimer5B_IRQHandler,          // Wide Timer 5 subtimer B
   (uint32_t)FPU_IRQHandler,                  // FPU
    0,                                        // Reserved
    0,                                        // Reserved
   (uint32_t)I2C4_IRQHandler,                 // I2C4 Master and Slave
   (uint32_t)I2C5_IRQHandler,                 // I2C5 Master and Slave
   (uint32_t)GPIOPortM_IRQHandler,            // GPIO Port M
   (uint32_t)GPIOPortN_IRQHandler,            // GPIO Port N
   (uint32_t)QEI2_IRQHandler,                 // Quadrature Encoder 2
    0,                                        // Reserved
    0,                                        // Reserved
   (uint32_t)GPIOPortP0_IRQHandler,           // GPIO Port P (Summary or P0)
   (uint32_t)GPIOPortP1_IRQHandler,           // GPIO Port P1
   (uint32_t)GPIOPortP2_IRQHandler,           // GPIO Port P2
   (uint32_t)GPIOPortP3_IRQHandler,           // GPIO Port P3
   (uint32_t)GPIOPortP4_IRQHandler,           // GPIO Port P4
   (uint32_t)GPIOPortP5_IRQHandler,           // GPIO Port P5
   (uint32_t)GPIOPortP6_IRQHandler,           // GPIO Port P6
   (uint32_t)GPIOPortP7_IRQHandler,           // GPIO Port P7
   (uint32_t)GPIOPortQ0_IRQHandler,           // GPIO Port Q (Summary or Q0)
   (uint32_t)GPIOPortQ1_IRQHandler,           // GPIO Port Q1
   (uint32_t)GPIOPortQ2_IRQHandler,           // GPIO Port Q2
   (uint32_t)GPIOPortQ3_IRQHandler,           // GPIO Port Q3
   (uint32_t)GPIOPortQ4_IRQHandler,           // GPIO Port Q4
   (uint32_t)GPIOPortQ5_IRQHandler,           // GPIO Port Q5
   (uint32_t)GPIOPortQ6_IRQHandler,           // GPIO Port Q6
   (uint32_t)GPIOPortQ7_IRQHandler,           // GPIO Port Q7
   (uint32_t)GPIOPortR_IRQHandler,            // GPIO Port R
   (uint32_t)GPIOPortS_IRQHandler,            // GPIO Port S
   (uint32_t)PWM1Gen0_IRQHandler,             // PWM 1 Generator 0
   (uint32_t)PWM1Gen1_IRQHandler,             // PWM 1 Generator 1
   (uint32_t)PWM1Gen2_IRQHandler,             // PWM 1 Generator 2
   (uint32_t)PWM1Gen3_IRQHandler,             // PWM 1 Generator 3
   (uint32_t)PWM1Fault_IRQHandler             // PWM 1 Fault
};

void Reset_Handler(void)
{
  //copy .data section from flash to SRAM
  uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;

  uint8_t *dst = (uint8_t*)&_sdata; //SRAM
  uint8_t *src = (uint8_t*)&_la_data; //flash 
  
  for(uint32_t i = 0; i < size; i++)
  {
    *dst++ = *src++;
    }

    //Init the .bss section with Zero in SRAM
  size = (uint32_t)&_ebss - (uint32_t)&_sbss;
  dst = (uint8_t*)&_sbss;

  for(uint32_t i = 0; i < size; i++)
  {
    *dst++ = 0;
    }

__libc_init_array();
  main();
}

void Default_Handler(void)
{
  while(1);
}




ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx) : ORIGIN =0x00000000, LENGTH = 256K /* 256KB Flash size */
  SRAM (rwx) : ORIGIN =0x20000000, LENGTH = 32K /* 32KB RAM size */
}


SECTIONS
{
  .text :
  {
    *(.isr_vector)
    *(.text)
    *(.text.*)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  _la_data = LOADADDR(.data);

  .data :
  {
    _sdata = .;
    *(.data)
    *(.data.*)
    . = ALIGN(4);
    _edata = .;
  } > SRAM AT > FLASH

  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
    . = ALIGN(4);
    end = .;
    __end__ = .;
  } > SRAM
}

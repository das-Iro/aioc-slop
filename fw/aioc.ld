/* STM32F302CB: 128 KB flash (last 2 KB page holds the settings), 32 KB RAM */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 126K
  SETT  (r)   : ORIGIN = 0x0801F800, LENGTH = 2K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K - 16
  NOINIT (rw) : ORIGIN = 0x20007FF0, LENGTH = 16
}

ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
_settings_page = ORIGIN(SETT);

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
  .bss (NOLOAD) : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
  .noinit (NOLOAD) : { *(.noinit*) } > NOINIT
  end = _ebss;
}

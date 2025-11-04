/* Linker script for the RP2040 */
MEMORY
{
  FLASH : ORIGIN = 0x10000000, LENGTH = 2M
  RAM : ORIGIN = 0x20000000, LENGTH = 256K
}

/* These values are provided by the build system */
EXTERN(__stack_start__);
EXTERN(__stack_end__);
EXTERN(__sdata);
EXTERN(__edata);
EXTERN(__sbss);
EXTERN(__ebss);
EXTERN(__srodata);
EXTERN(__erodata);
EXTERN(__text_start);
EXTERN(__text_end);
EXTERN(__uninit_start);
EXTERN(__uninit_end);

/* Add __sidata */
__sidata = ORIGIN(FLASH) + LENGTH(FLASH);

SECTIONS
{
  .text :
  {
    . = ALIGN(4);
    KEEP(*(.vector_table.reset_vector));
    *(.text .text.*);
    *(.rodata .rodata.*);
    . = ALIGN(4);
  } > FLASH

  .data :
  {
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    __edata = .; 
  } > RAM AT > FLASH

  .bss :
  {
    . = ALIGN(4);
    __sbss = .; 
    *(.bss .bss.*);
    __ebss = .; 
  } > RAM

  .uninit :
  {
    . = ALIGN(4);
    __uninit_start = .; 
    *(.uninit .uninit.*);
    __uninit_end = .; 
  } > RAM

  .stack :
  {
    . = ALIGN(8);
    __stack_start__ = .; 
    *(.stack .stack.*);
    __stack_end__ = .; 
  } > RAM

  /DISCARD/ :
  {
    *(.debug*);
    *(.comment);
    *(.ARM.attributes);
  }
}
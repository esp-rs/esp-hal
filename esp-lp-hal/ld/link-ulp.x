/* NOTE: Adapted from picorv32-rt/link.x */

ENTRY(_initjmp)

CONFIG_ULP_COPROC_RESERVE_MEM = 8 * 1024;

MEMORY
{
    RAM(RW) : ORIGIN = 0, LENGTH = CONFIG_ULP_COPROC_RESERVE_MEM
}

PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

/* # Abort function */
/* Allow user to optionally override the default abort implementation. */
PROVIDE(abort = default_abort);

/* Default exception/interrupt handlers, may be overridden by user.
 * TODO: Improve interrupt ergonomics.
*/;
PROVIDE(TimerInterrupt = default_timer_interrupt);
PROVIDE(IllegalInstructionException = default_exception_handler);
PROVIDE(BusErrorException = default_exception_handler);
PROVIDE(SensInterrupt = default_sens_interrupt);
PROVIDE(GpioInterrupt = default_gpio_interrupt);


SECTIONS
{
  . = ORIGIN(RAM);

  .text :
  {
    /* Put reset handler first in .text section so it ends up 
     * as the entry point of the program.
     */
    KEEP(*(.initjmp));
    . = ALIGN(0x10);
    KEEP(*(.trap));
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.trap.rust));
    *(.text .text.*)
  } >RAM

  .rodata ALIGN(4):
  {
    *(.rodata .rodata.*)
  } >RAM

  .data ALIGN(4):
  {
    /* Must be called __global_pointer$ for linker relaxations to work. */
    PROVIDE(__global_pointer$ = . + 0x800);
    *(.data .data.*)
    *(.sdata .sdata.*)
  } >RAM

  .bss ALIGN(4):
  {
    *(.bss .bss.*)
    *(.sbss .sbss.*)
  } >RAM


  /* fictitious region that represents the memory available for the stack */
  .stack (NOLOAD):
  {
    . = ALIGN(4);
    . = _stack_start;
  } >RAM

  /* fake output .got section */
  /* Dynamic relocations are unsupported. This section is only used to detect
     relocatable code in the input files and raise an error if relocatable code
     is found */
  .got (INFO) :
  {
    KEEP(*(.got .got.*));
  }

  /* Discard .eh_frame, we are not doing unwind on panic so it is not needed */
  /DISCARD/ :
  {
    *(.eh_frame);
  }
}

/* Do not exceed this mark in the error messages below                | */
ASSERT(SIZEOF(.got) == 0, "
.got section detected in the input files. Dynamic relocations are not
supported. If you are linking to C code compiled using the `gcc` crate
then modify your build script to compile the C code _without_ the
-fPIC flag. See the documentation of the `gcc::Config.fpic` method for
details.");

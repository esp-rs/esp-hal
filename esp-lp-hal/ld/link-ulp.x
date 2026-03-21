/* NOTE: Adapted from riscv_rt/link.x */
INCLUDE memory.x
INCLUDE interrupts.x
INCLUDE exceptions.x

/* Default abort entry point. If no abort symbol is provided, then abort maps to _default_abort. */
EXTERN(_default_abort);
PROVIDE(abort = _default_abort);

/* Trap for exceptions triggered during initialization. If the execution reaches this point, it
   means that there is a bug in the boot code. If no _pre_init_trap symbol is provided, then
  _pre_init_trap defaults to _default_abort. Note that _pre_init_trap must be 4-byte aligned */
PROVIDE(_pre_init_trap = _default_abort);

/* Multi-processor hook function (for multi-core targets only). If no _mp_hook symbol
   is provided, then _mp_hook maps to _default_mp_hook, which leaves HART 0 running while
   the other HARTS stuck in a busy loop. Note that _default_mp_hook cannot be overwritten.
   We use PROVIDE to avoid compilation errors in single hart targets, not to allow users
   to overwrite the symbol. */
PROVIDE(_default_mp_hook = abort);
PROVIDE(_mp_hook = _default_mp_hook);

/* Default trap entry point. If not _start_trap symbol is provided, then _start_trap maps to
   _default_start_trap, which saves caller saved registers, calls _start_trap_rust, restores
   caller saved registers and then returns. Note that _start_trap must be 4-byte aligned */
/* EXTERN(_default_start_trap);*/
/* PROVIDE(_start_trap = _default_start_trap); */

/* Default interrupt setup entry point. If not _setup_interrupts symbol is provided, then
   _setup_interrupts maps to _default_setup_interrupts, which in direct mode sets the value
   of the xtvec register to _start_trap and, in vectored mode, sets its value to
   _vector_table and enables vectored mode. */
/* EXTERN(_default_setup_interrupts); */
PROVIDE(_setup_interrupts = _setup_interrupts);

/* Default main routine. If no hal_main symbol is provided, then hal_main maps to main, which
   is usually defined by final users via the #[riscv_rt::entry] attribute. Using hal_main
   instead of main directly allow HALs to inject code before jumping to user main. */
/* PROVIDE(hal_main = main); */

/* riscv-rt will jump to the entrypoint of esp-lp-hal, which is rust_main */
/* rust_main will then jumps to main() */
/* which itself is wrapped using esp-lp-hal::entry, to add the ULP_MAGIC symbols lol. */
/* SO the full boot procedure is... */
/* reset_vector --> _start --> _start_rust + (_setup_interrupts) --> $hal_main --> main */
PROVIDE(hal_main = rust_main);

/* Default exception handler. By default, the exception handler is abort.
   Users can override this alias by defining the symbol themselves */
PROVIDE(ExceptionHandler = abort);

/* Default interrupt handler. By default, the interrupt handler is abort.
   Users can override this alias by defining the symbol themselves */
PROVIDE(DefaultHandler = abort);

/* Default interrupt trap entry point. When vectored trap mode is enabled,
   the riscv-rt crate provides an implementation of this function, which saves caller saved
   registers, calls the the DefaultHandler ISR, restores caller saved registers and returns.
   Note, however, that this provided implementation cannot be overwritten. We use PROVIDE
   to avoid compilation errors in direct mode, not to allow users to overwrite the symbol. */
PROVIDE(_start_DefaultHandler_trap = _start_trap);

/* # Pre-initialization function */
/* If the user overrides this using the `#[pre_init]` attribute or by creating a `__pre_init` function,
   then the function this points to will be called before the RAM is initialized. */
PROVIDE(__pre_init = default_pre_init);

SECTIONS
{
  PROVIDE(_stext = ORIGIN(RAM));

  .text ALIGN(_stext,4) :
  {
    /*
     * Default reset vector must link to offset 0x0,
     * and interrupt handler must link to offset 0x10.
     */
    KEEP(*(.init.vectors));
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.trap.rust));
    *(.text.abort);
    *(.text .text.*);

    . = ALIGN(4);
    __etext = .;
  } > REGION_TEXT

  .rodata ALIGN(4) :
  {
    *(.rodata .rodata.*);
  } > REGION_RODATA

  .bss :
  {
    __sbss = .;
    *(.bss .bss.*);
    . = ALIGN(4);
    __ebss = .;
  } > REGION_BSS

  .data : AT(LOADADDR(.rodata) + SIZEOF(.rodata))
  {
    __sidata = LOADADDR(.data);
    __sdata = .;
    /* Must be called __global_pointer$ for linker relaxations to work. */
    PROVIDE(__global_pointer$ = . + 0x800);
    *(.data .data.*);
    . = ALIGN(4);
    __edata = .;
  } > RAM

  PROVIDE(_heap_size = 0);

  /* fictitious region that represents the memory available for the heap */
  .heap (NOLOAD) :
  {
    __sheap = .;
    . += _heap_size;
    . = ALIGN(4);
    __eheap = .;
  } > RAM

  /* fictitious region that represents the memory available for the stack */
  .stack (NOLOAD) :
  {
    __estack = .;
    . = _stack_start;
    __sstack = .;
  } > RAM

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

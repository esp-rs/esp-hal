ENTRY(_start)

/*
We don't use the linker scripts defined in riscv-rt (since we have special needs) but we need to define some symbols to satisfy riscv-rt
*/

PROVIDE(_stext = ORIGIN(ROTEXT));
PROVIDE(__global_pointer$ = ALIGN(_data_start, 4) + 0x800);

/* Default abort entry point. If no abort symbol is provided, then abort maps to _default_abort. */
EXTERN(_default_abort);
PROVIDE(abort = _default_abort);

/* Trap for exceptions triggered during initialization. If the execution reaches this point, it
   means that there is a bug in the boot code. If no _pre_init_trap symbol is provided, then
  _pre_init_trap defaults to _default_abort. Note that _pre_init_trap must be 4-byte aligned */
PROVIDE(_pre_init_trap = _default_abort);

/* Default trap entry point. If not _start_trap symbol is provided, then _start_trap maps to
   _default_start_trap, which saves caller saved registers, calls _start_trap_rust, restores
   caller saved registers and then returns. Note that _start_trap must be 4-byte aligned */
EXTERN(_default_start_trap);
PROVIDE(_start_trap = _default_start_trap);

/* Default interrupt setup entry point. If not _setup_interrupts symbol is provided, then
   _setup_interrupts maps to _default_setup_interrupts, which in direct mode sets the value
   of the xtvec register to _start_trap and, in vectored mode, sets its value to
   _vector_table and enables vectored mode. */
EXTERN(_default_setup_interrupts);
PROVIDE(_setup_interrupts = _default_setup_interrupts);

/* Default main routine. If no hal_main symbol is provided, then hal_main maps to main, which
   is usually defined by final users via the #[riscv_rt::entry] attribute. Using hal_main
   instead of main directly allow HALs to inject code before jumping to user main. */
PROVIDE(hal_main = main);

/* Default exception handler. By default, the exception handler is abort.
   Users can override this alias by defining the symbol themselves */
PROVIDE(ExceptionHandler = abort);

/* Default interrupt trap entry point. When vectored trap mode is enabled,
   the riscv-rt crate provides an implementation of this function, which saves caller saved
   registers, calls the the DefaultHandler ISR, restores caller saved registers and returns.
   Note, however, that this provided implementation cannot be overwritten. We use PROVIDE
   to avoid compilation errors in direct mode, not to allow users to overwrite the symbol. */
PROVIDE(_start_DefaultHandler_trap = _start_trap);

/* Default interrupt handler. */
PROVIDE(DefaultHandler = EspDefaultHandler);

PROVIDE(_max_hart_id = 0);

/* don't init data - expect the bootloader to do it */
__sdata = 0;
__edata = 0;
__sidata = 0;

/* alias bss start + end as expected by riscv-rt */
__sbss = _bss_start;
__ebss = _bss_end;

PROVIDE(interrupt1 = DefaultHandler);
PROVIDE(interrupt2 = DefaultHandler);
PROVIDE(interrupt3 = DefaultHandler);
PROVIDE(interrupt4 = DefaultHandler);
PROVIDE(interrupt5 = DefaultHandler);
PROVIDE(interrupt6 = DefaultHandler);
PROVIDE(interrupt7 = DefaultHandler);
PROVIDE(interrupt8 = DefaultHandler);
PROVIDE(interrupt9 = DefaultHandler);
PROVIDE(interrupt10 = DefaultHandler);
PROVIDE(interrupt11 = DefaultHandler);
PROVIDE(interrupt12 = DefaultHandler);
PROVIDE(interrupt13 = DefaultHandler);
PROVIDE(interrupt14 = DefaultHandler);
PROVIDE(interrupt15 = DefaultHandler);
PROVIDE(interrupt16 = DefaultHandler);
PROVIDE(interrupt17 = DefaultHandler);
PROVIDE(interrupt18 = DefaultHandler);
PROVIDE(interrupt19 = DefaultHandler);
PROVIDE(interrupt20 = DefaultHandler);
PROVIDE(interrupt21 = DefaultHandler);
PROVIDE(interrupt22 = DefaultHandler);
PROVIDE(interrupt23 = DefaultHandler);
PROVIDE(interrupt24 = DefaultHandler);
PROVIDE(interrupt25 = DefaultHandler);
PROVIDE(interrupt26 = DefaultHandler);
PROVIDE(interrupt27 = DefaultHandler);
PROVIDE(interrupt28 = DefaultHandler);
PROVIDE(interrupt29 = DefaultHandler);
PROVIDE(interrupt30 = DefaultHandler);
PROVIDE(interrupt31 = DefaultHandler);

/* external interrupts (from PAC) */
INCLUDE "device.x"

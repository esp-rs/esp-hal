ENTRY(_start)

PROVIDE(_stext = ORIGIN(ROTEXT));
PROVIDE(_stack_start = ORIGIN(RWDATA) + LENGTH(RWDATA));
PROVIDE(_max_hart_id = 0);
PROVIDE(_hart_stack_size = 2K);
PROVIDE(_heap_size = 0);

PROVIDE(UserSoft = DefaultHandler);
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(UserTimer = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(UserExternal = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

PROVIDE(DefaultHandler = DefaultInterruptHandler);
PROVIDE(ExceptionHandler = DefaultExceptionHandler);

/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, while the ESP32-C6 has
   IDs 0-31, so we much define the handler for the one additional interrupt
   ID: */
PROVIDE(interrupt0 = DefaultHandler);

PROVIDE(__post_init = default_post_init);

/* A PAC/HAL defined routine that should initialize custom interrupt controller if needed. */
PROVIDE(_setup_interrupts = default_setup_interrupts);

/* # Multi-processing hook function
   fn _mp_hook() -> bool;

   This function is called from all the harts and must return true only for one hart,
   which will perform memory initialization. For other harts it must return false
   and implement wake-up in platform-dependent way (e.g. after waiting for a user interrupt).
*/
PROVIDE(_mp_hook = default_mp_hook);

/* # Start trap function override
  By default uses the riscv crates default trap handler
  but by providing the `_start_trap` symbol external crates can override.
*/
PROVIDE(_start_trap = default_start_trap);

/* esp32c6 fixups */
/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, while the ESP32-C6 has
   IDs 0-31, so we much define the handler for the one additional interrupt
   ID: */
PROVIDE(interrupt0 = DefaultHandler);

/* Must be called __global_pointer$ for linker relaxations to work. */
PROVIDE(__global_pointer$ = _data_start + 0x800);

SECTIONS {
  /* These symbols/functions need to be near eachother, group them together at the start of text */
  .text_init _stext : ALIGN(4)
  {
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.text.abort));
    KEEP(*(.trap));
    KEEP(*(.trap.rust));
  } > ROTEXT
}
INSERT BEFORE .text;

SECTIONS {
  /**
   * Bootloader really wants to have separate segments for ROTEXT and RODATA
   * Thus, we need to force a gap here.
   */
  .text_gap (NOLOAD): {
    . = . + 4;
    . = ALIGN(4) + 0x20;
  } > ROM
}
INSERT BEFORE .rodata;
/* end of esp32c6 fixups */

/* Shared sections - ordering matters */
INCLUDE "text.x"
INCLUDE "rodata.x"
INCLUDE "rwdata.x"
INCLUDE "rwtext.x"
INCLUDE "rtc_fast.x"
/* End of Shared sections */

INCLUDE "debug.x"
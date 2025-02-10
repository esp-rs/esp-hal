ENTRY(_start)

PROVIDE(_stext = ORIGIN(ROTEXT));
PROVIDE(_max_hart_id = 0);

PROVIDE(UserSoft = DefaultHandler);
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(UserTimer = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(UserExternal = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

PROVIDE(ExceptionHandler = DefaultExceptionHandler);

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

/* esp32c3 fixups */

SECTIONS {
  .trap : ALIGN(4)
  {
    KEEP(*(.trap));
    *(.trap.*);
  } > RWTEXT
}
INSERT BEFORE .rwtext;

SECTIONS {
  .rotext_dummy (NOLOAD) :
  {
    /* This dummy section represents the .rodata section but in ROTEXT.
     * Thus, it must have its alignment and (at least) its size.
     */

    /* Start at the same alignment constraint than .flash.text */

    . = ALIGN(ALIGNOF(.rodata));
    . = ALIGN(ALIGNOF(.rodata.wifi));

    /* Create an empty gap as big as .text section */

    . = . + SIZEOF(.rodata_desc);
    . = . + SIZEOF(.rodata);
    . = . + SIZEOF(.rodata.wifi);

    /* Prepare the alignment of the section above. Few bytes (0x20) must be
     * added for the mapping header.
     */

    . = ALIGN(0x10000) + 0x20;
    _rotext_reserved_start = .;
  } > ROTEXT
}
INSERT BEFORE .text;

/* Similar to .rotext_dummy this represents .rwtext but in .data */
SECTIONS {
  .rwdata_dummy (NOLOAD) : ALIGN(4)
  {
    . = . + SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi) + SIZEOF(.trap);
  } > RWDATA
}
INSERT BEFORE .data;

/* Must be called __global_pointer$ for linker relaxations to work. */
PROVIDE(__global_pointer$ = _data_start + 0x800);
/* end of esp32c3 fixups */

/* Shared sections - ordering matters */
SECTIONS {
  INCLUDE "rodata_desc.x"
  INCLUDE "rwtext.x"
  INCLUDE "rwdata.x"
}
INCLUDE "rodata.x"
INCLUDE "text.x"
INCLUDE "rtc_fast.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
/* End of Shared sections */

INCLUDE "debug.x"

_dram_origin = ORIGIN( DRAM );

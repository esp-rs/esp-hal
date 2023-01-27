ENTRY(_start)

PROVIDE(_stext = ORIGIN(REGION_TEXT));
PROVIDE(_stack_start = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK));
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

/* # Pre-initialization function */
/* If the user overrides this using the `#[pre_init]` attribute or by creating a `__pre_init` function,
   then the function this points to will be called before the RAM is initialized. */
PROVIDE(__pre_init = default_pre_init);

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

SECTIONS
{
  .text.dummy (NOLOAD) :
  {
    /* This section is intended to make _stext address work */
    . = ABSOLUTE(_stext);
  } > REGION_TEXT

  .text _stext :
  {
    _stext = .;
    /* Put reset handler first in .text section so it ends up as the entry */
    /* point of the program. */
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.text.abort));
    . = ALIGN(4);
    KEEP(*(.trap));
    KEEP(*(.trap.rust));

    *(.text .text.*);
    _etext = .;
  } > REGION_TEXT

  /**
   * This dummy section represents the .text section but in rodata.
   * Thus, it must have its alignement and (at least) its size.
   */
  .text_dummy (NOLOAD):
  {
    /* Start at the same alignement constraint than .text */
    . = ALIGN(ALIGNOF(.text));
    /* Create an empty gap as big as .text section */
    . = . + SIZEOF(.text);
    /* Prepare the alignement of the section above. Few bytes (0x20) must be
     * added for the mapping header. */
    . = ALIGN(0x10000) + 0x20;
  } > REGION_RODATA

  .rodata : ALIGN(4)
  {
    _srodata = .;
    *(.srodata .srodata.*);
    *(.rodata .rodata.*);

    /* 4-byte align the end (VMA) of this section.
       This is required by LLD to ensure the LMA of the following .data
       section will have the correct alignment. */
    . = ALIGN(4);
    _erodata = .;
  } > REGION_RODATA

  .rwtext : ALIGN(4) {
    _irwtext = LOADADDR(.rwtext);
    _srwtext = .;
    *(.rwtext);

    *(.iram1)
    *(.iram1.*)

    *(.wifi0iram .wifi0iram.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)

    . = ALIGN(4);
    _erwtext = .;
  } > REGION_RWTEXT

  /* similar as text_dummy */
  .ram_dummy (NOLOAD) : {
    . = ALIGN(ALIGNOF(.rwtext));
    . = . + SIZEOF(.rwtext);
  } > REGION_DATA

  .data : ALIGN(8)
  {
    _sidata = LOADADDR(.data);
    _sdata = .;
    /* Must be called __global_pointer$ for linker relaxations to work. */
    PROVIDE(__global_pointer$ = . + 0x800);
    *(.sdata .sdata.* .sdata2 .sdata2.*);
    *(.data .data.*);
    *(.data1)

    . = ALIGN(8);
    _edata = .;
  } > REGION_DATA

  .bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sbss = .;
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.sbss .sbss.* .bss .bss.*);
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)

    . = ALIGN(8);
    _ebss = .;
  } > REGION_BSS

  /* fictitious region that represents the memory available for the heap */
  .heap (NOLOAD) :
  {
    _sheap = .;
    _heap_start = .;
    . += _heap_size;
    . = ALIGN(4);
    _eheap = .;
  } > REGION_HEAP

  /* fictitious region that represents the memory available for the stack */
  .stack (NOLOAD) :
  {
    _estack = .;
    . = ABSOLUTE(_stack_start);
    _sstack = .;
  } > REGION_STACK

  .eh_frame (INFO) : { KEEP(*(.eh_frame)) }
  .eh_frame_hdr (INFO) : { *(.eh_frame_hdr) }
}

/* Do not exceed this mark in the error messages above                                    | */
ASSERT(ORIGIN(REGION_TEXT) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_TEXT must be 4-byte aligned");

ASSERT(ORIGIN(REGION_RODATA) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_RODATA must be 4-byte aligned");

ASSERT(ORIGIN(REGION_DATA) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_DATA must be 4-byte aligned");

ASSERT(ORIGIN(REGION_HEAP) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_HEAP must be 4-byte aligned");

ASSERT(ORIGIN(REGION_TEXT) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_TEXT must be 4-byte aligned");

ASSERT(ORIGIN(REGION_STACK) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_STACK must be 4-byte aligned");

ASSERT(_stext % 4 == 0, "
ERROR(riscv-rt): `_stext` must be 4-byte aligned");

ASSERT(_sdata % 4 == 0 && _edata % 4 == 0, "
BUG(riscv-rt): .data is not 4-byte aligned");

ASSERT(_sidata % 4 == 0, "
BUG(riscv-rt): the LMA of .data is not 4-byte aligned");

ASSERT(_sbss % 4 == 0 && _ebss % 4 == 0, "
BUG(riscv-rt): .bss is not 4-byte aligned");

ASSERT(_sheap % 4 == 0, "
BUG(riscv-rt): start of .heap is not 4-byte aligned");

ASSERT(_stext + SIZEOF(.text) < ORIGIN(REGION_TEXT) + LENGTH(REGION_TEXT), "
ERROR(riscv-rt): The .text section must be placed inside the REGION_TEXT region.
Set _stext to an address smaller than 'ORIGIN(REGION_TEXT) + LENGTH(REGION_TEXT)'");

ASSERT(SIZEOF(.stack) > (_max_hart_id + 1) * _hart_stack_size, "
ERROR(riscv-rt): .stack section is too small for allocating stacks for all the harts.
Consider changing `_max_hart_id` or `_hart_stack_size`.");

ASSERT(SIZEOF(.got) == 0, "
.got section detected in the input files. Dynamic relocations are not
supported. If you are linking to C code compiled using the `gcc` crate
then modify your build script to compile the C code _without_ the
-fPIC flag. See the documentation of the `gcc::Config.fpic` method for
details.");

/* Do not exceed this mark in the error messages above                                    | */


/* before memory.x to allow override */
ENTRY(Reset)

INCLUDE memory.x

/* after memory.x to allow override */
PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);

INCLUDE exception.x

/* map generic regions to output sections */
INCLUDE "alias.x"

/* Fixups for esp32s2 */
SECTIONS {
  .iram0_reserved_for_text (NOLOAD) : ALIGN(4)
  {
    . = ORIGIN(RWDATA) + SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi);
  } > RWDATA
}
INSERT BEFORE .data;
/* End of fixups for esp32s2 */

/* Shared sections - ordering matters */
INCLUDE "text.x"
INCLUDE "rodata.x"
INCLUDE "rwtext.x"
INCLUDE "rwdata.x"
INCLUDE "rtc_fast.x"
INCLUDE "rtc_slow.x"
INCLUDE "external.x"
/* End of Shared sections */

_heap_end = ABSOLUTE(ORIGIN(dram_seg))+LENGTH(dram_seg)-LENGTH(reserved_for_boot_seg) - STACK_SIZE;

_stack_start_cpu0 = _heap_end;
_stack_end_cpu0 = _stack_start_cpu0 + STACK_SIZE;

EXTERN(DefaultHandler);

INCLUDE "device.x"

/* before memory.x to allow override */
ENTRY(ESP32Reset)

/* after memory.x to allow override */
PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);
PROVIDE(__post_init = default_post_init);

INCLUDE exception.x

/* Fixups for esp32s2 */
SECTIONS {
  .rwdata_dummy (NOLOAD) : ALIGN(4)
  {
    . = ORIGIN(RWDATA) + SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi);
  } > RWDATA
}
INSERT BEFORE .data;

INCLUDE "fixups/rtc_fast_rwdata_dummy.x"
/* End of fixups for esp32s2 */

/* Shared sections - ordering matters */
INCLUDE "text.x"
INCLUDE "rodata.x"
INCLUDE "rwtext.x"
INCLUDE "rwdata.x"
INCLUDE "rtc_fast.x"
INCLUDE "rtc_slow.x"
/* End of Shared sections */

_stack_region_top = ABSOLUTE(ORIGIN(dram_seg))+LENGTH(dram_seg);
_stack_region_bottom = _stack_end;

_stack_end_cpu0 = _stack_region_top;
_stack_start_cpu0 = _stack_region_bottom;

EXTERN(DefaultHandler);

INCLUDE "device.x"


/* before memory.x to allow override */
ENTRY(ESP32Reset)

/* after memory.x to allow override */
PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);
PROVIDE(__post_init = default_post_init);

PROVIDE(__level_1_interrupt = handle_interrupts);
PROVIDE(__level_2_interrupt = handle_interrupts);
PROVIDE(__level_3_interrupt = handle_interrupts);

INCLUDE exception.x

/* This represents .rwtext but in .data */
SECTIONS {
  .rwdata_dummy (NOLOAD) : ALIGN(4)
  {
    . = . + SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi);
  } > RWDATA
}
INSERT BEFORE .data;

INCLUDE "fixups/rtc_fast_rwdata_dummy.x"
/* End of fixups for esp32s2 */

/* Shared sections - ordering matters */
SECTIONS {
  INCLUDE "rodata_desc.x"
  INCLUDE "rwtext.x"
  INCLUDE "rwdata.x"
}
INCLUDE "rodata.x"
INCLUDE "text.x"
INCLUDE "rtc_fast.x"
INCLUDE "rtc_slow.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
/* End of Shared sections */

EXTERN(DefaultHandler);

INCLUDE "device.x"

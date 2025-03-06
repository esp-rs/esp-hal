
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

/* ESP32 fixups */
INCLUDE "fixups/rtc_fast_rwdata_dummy.x"
/* END ESP32 fixups */

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

EXTERN(WIFI_EVENT); /* Force inclusion of WiFi libraries */

INCLUDE "device.x"

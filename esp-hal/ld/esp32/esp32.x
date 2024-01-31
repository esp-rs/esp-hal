
/* before memory.x to allow override */
ENTRY(ESP32Reset)

/* after memory.x to allow override */
PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);
PROVIDE(__post_init = default_post_init);

INCLUDE exception.x

/* ESP32 fixups */
INCLUDE "fixups/rtc_fast_rwdata_dummy.x"
/* END ESP32 fixups */

/* Shared sections - ordering matters */
INCLUDE "text.x"
INCLUDE "rodata.x"
INCLUDE "rwtext.x"
INCLUDE "rwdata.x"
INCLUDE "rtc_fast.x"
INCLUDE "rtc_slow.x"
INCLUDE "stack.x"
/* End of Shared sections */

/* an uninitialized section for use as the wifi-heap in esp-wifi */
SECTIONS {
    .dram2_uninit (NOLOAD) : ALIGN(4) {
        *(.dram2_uninit)
    } > dram2_seg
}

EXTERN(DefaultHandler);

EXTERN(WIFI_EVENT); /* Force inclusion of WiFi libraries */

INCLUDE "device.x"

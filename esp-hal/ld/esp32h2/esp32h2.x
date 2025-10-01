/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, while the ESP32-C6 and ESP32-H2 have
   IDs 0-31, so we much define the handler for the one additional interrupt
   ID: */
PROVIDE(interrupt0 = DefaultHandler);

SECTIONS {
  /* Shared sections - ordering matters */
  INCLUDE "rwtext.x"
  INCLUDE "rwdata.x"
  /* End of Shared sections */
}
#IF ESP_HAL_CONFIG_FLIP_LINK
/* INSERT BEFORE does not seem to work for the .stack section. Instead, we place every RAM
  section after .stack if `flip_link` is enabled. */
INSERT AFTER .stack;
#ENDIF

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
INSERT BEFORE .text;

/* Shared sections #2 - ordering matters */
INCLUDE "rodata.x"
INCLUDE "text.x"
INCLUDE "rtc_fast.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
INCLUDE "metadata.x"
INCLUDE "eh_frame.x"
/* End of Shared sections #2 */

_dram_origin = ORIGIN( RAM );

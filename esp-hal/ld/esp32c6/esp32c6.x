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

/* esp32c6 fixups */
/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, while the ESP32-C6 has
   IDs 0-31, so we much define the handler for the one additional interrupt
   ID: */
PROVIDE(interrupt0 = DefaultHandler);

SECTIONS {
  .trap : ALIGN(4)
  {
    KEEP(*(.trap));
    *(.trap.*);
  } > RWTEXT

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
/* end of esp32c6 fixups */

/* Shared sections #2 - ordering matters */
SECTIONS {
  INCLUDE "rodata_desc.x"
}

INCLUDE "rodata.x"
INCLUDE "text.x"
INCLUDE "rtc_fast.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
/* End of Shared sections #2 */

INCLUDE "debug.x"

_dram_origin = ORIGIN( RAM );

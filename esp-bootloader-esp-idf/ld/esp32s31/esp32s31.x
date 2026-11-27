PROVIDE(_max_hart_id = 1);

/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, while the ESP32-S31 has
   IDs 0-31 (CLIC), so we must define the handler for the one additional
   interrupt ID: */
PROVIDE(interrupt0 = DefaultHandler);

/* # Multi-processing hook function
   fn _mp_hook() -> bool;
   This function is called from all the harts and must return true only for one hart,
   which will perform memory initialization. For other harts it must return false
   and implement wake-up in platform-dependent way (e.g. after waiting for a user interrupt).
*/
PROVIDE(_mp_hook = default_mp_hook);

/* `psram.x` must come before `rwdata.x`. See `psram.x`. */
INCLUDE "psram.x"

SECTIONS {
  /* Shared sections - ordering matters */
  INCLUDE "rwtext.x"
  INCLUDE "rwdata.x"
  /* End of Shared sections */
}

SECTIONS {
  /**
   * The ESP-IDF bootloader takes exactly two mapped segments. Without code in PSRAM, the gap
   * makes ROTEXT and RODATA two segments. With code in PSRAM, the start-up in the flash window is
   * the second segment, so ROTEXT and RODATA must be one segment, without a gap.
   */
  .text_gap (NOLOAD): {
    . = . + (ESP_HAL_CODE_IN_PSRAM ? 0 : 8);
    . = ALIGN(4) + (ESP_HAL_CODE_IN_PSRAM ? 0 : 0x20);
  } > ROM
}
INSERT BEFORE .text;

INCLUDE "rodata.x"
INCLUDE "text.x"
ASSERT(!ESP_HAL_CODE_IN_PSRAM || ADDR(.text) == ADDR(.rodata.wifi) + SIZEOF(.rodata.wifi), "With code-in-psram, .text must start where .rodata.wifi ends");
INCLUDE "rtc_fast.x"
/* See `psram_code_end.x`. The sections below are in RAM, or are INFO sections at 0. */
INCLUDE "psram_code_end.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
INCLUDE "metadata.x"
INCLUDE "eh_frame.x"
/* End of Shared sections */

_dram_data_start = ABSOLUTE(ADDR(.rwtext) + SIZEOF(.rwtext));

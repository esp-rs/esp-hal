INCLUDE "trap.x"

INCLUDE "esp_hal_rwtext_sections.x"

.rwtext : ALIGN(4)
{
  . = ALIGN (4);

  /* Keep each object's literal pool adjacent to its code (see comment in `text.x`). */
  *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)
  /* unconditionally add patched SPI-flash ROM functions (from esp-rom-sys) - the linker is still happy if there are none */
  *:esp_rom_spiflash.*(.literal .text .literal.* .text.*)

  INCLUDE "esp_hal_rwtext.x"

  . = ALIGN(4);
} > RWTEXT

.rwtext.wifi :
{
  . = ALIGN(4);
  *( .wifi0iram  .wifi0iram.*)
  *( .wifirxiram  .wifirxiram.*)
  *( .wifislprxiram  .wifislprxiram.*)
  *( .wifislpiram  .wifislpiram.*)
  *( .phyiram  .phyiram.*)
  *( .iram1  .iram1.*)
  *( .wifiextrairam.* )
  *( .coexiram.* )
  *( .high_perf_code_iram* )
  *( .coexsleepiram* )
  *( .wifiorslpiram* )
  *( .isr_iram* )
  *( .conn_iram* )
  *( .sleep_iram* )
  . = ALIGN(4);

  _rwtext_end = .;
  _rwtext_len = ABSOLUTE(. - ORIGIN(RWTEXT));
} > RWTEXT

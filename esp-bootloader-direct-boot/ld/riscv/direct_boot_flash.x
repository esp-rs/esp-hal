/* Code and read-only data execute in place. The ROM maps flash offset 0 at `IROM_BASE` and
   `DROM_BASE`, so each section's address is the base plus its flash offset. They follow the RAM
   section images, starting at `_direct_boot_flash_lma`. */

SECTIONS {
  .text IROM_BASE + ALIGN(_direct_boot_flash_lma, 16) : AT(ALIGN(_direct_boot_flash_lma, 16))
  {
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.text.abort));
    INCLUDE "esp_hal_text_radio.x"
    *(.literal .text .literal.* .text.*)
    . = ALIGN(4);
  } > ROTEXT

  .rodata ALIGN(DROM_BASE + LOADADDR(.text) + SIZEOF(.text), 16) : AT(ADDR(.rodata) - DROM_BASE)
  {
    _rodata_start = ABSOLUTE(.);
    *(.rodata .rodata.*)
    *(.srodata .srodata.*)
    *(.rodata_wlog_*.*)
    . = ALIGN(4);
    _rodata_end = ABSOLUTE(.);
  } > RODATA
}

ASSERT(ADDR(.text) % ALIGNOF(.text) == 0, "
ERROR(esp-bootloader-direct-boot): .text is not aligned, raise the alignment of its flash offset");
ASSERT(ADDR(.rodata) % ALIGNOF(.rodata) == 0, "
ERROR(esp-bootloader-direct-boot): .rodata is not aligned, raise the alignment of its flash offset");

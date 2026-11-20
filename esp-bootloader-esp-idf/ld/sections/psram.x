/* Statics from `#[psram(zeroed)]`, zeroed by the start-up after it copies the code.
   This file must be included before `rwdata.x`, whose `.bss` takes `.bss.*`: the first matching
   output section takes an input section. The address is a forward reference to the end of
   everything in `ROM`, which the linker resolves because `PSRAM_BSS` has its own location
   counter. */
SECTIONS {
  .bss.psram ALIGN(_psram_code_end, 0x10000) (NOLOAD) :
  {
    _psram_bss_start = ABSOLUTE(.);
    *(.bss.psram .bss.psram.*)
    . = ALIGN(4);
    _psram_bss_end = ABSOLUTE(.);
  } > PSRAM_BSS
}

/* Only code in PSRAM gives PSRAM a fixed address at link time. */
ASSERT(ESP_HAL_CODE_IN_PSRAM || SIZEOF(.bss.psram) == 0, "Statics in PSRAM need code in PSRAM.");

/* The MMU page size. */
_psram_page_size = 0x10000;

/* Statics from `#[psram(zeroed)]`, zeroed by the start-up after it copies the code. Include this
   file before `rwdata.x`: the first matching output section takes an input section, and `.bss`
   there takes `.bss.*`. The address is a forward reference to the end of `ROM`. The linker
   resolves it because `PSRAM_BSS` has its own location counter. */
SECTIONS {
  .bss.psram ALIGN(_psram_code_end, _psram_page_size) (NOLOAD) :
  {
    _psram_bss_start = ABSOLUTE(.);
    *(.bss.psram .bss.psram.*)
    . = ALIGN(4);
    _psram_bss_end = ABSOLUTE(.);
  } > PSRAM_BSS
}

/* Only code in PSRAM gives PSRAM a fixed address at link time. `#[psram]` fails to compile
   without it, so this check is for a hand-written `link_section`. */
ASSERT(ESP_HAL_CODE_IN_PSRAM || SIZEOF(.bss.psram) == 0, "Statics in PSRAM need code in PSRAM.");

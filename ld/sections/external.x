

SECTIONS {
  .external.data :
  {
    _external_data_start = ABSOLUTE(.);
    . = ALIGN(4);
    *(.external.data .external.data.*)
    _external_data_end = ABSOLUTE(.);
  } > psram_seg AT > RODATA

  .external.bss (NOLOAD) :
  {
    _external_bss_start = ABSOLUTE(.);
    . = ALIGN(4);
    *(.external.bss .external.bss.*)
    _external_bss_end = ABSOLUTE(.);
  } > psram_seg

  .external.noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.external.noinit .external.noinit.*)
  } > psram_seg

  /* must be last segment using psram_seg */
  .external_heap_start (NOLOAD) :
  {
    . = ALIGN (4);
    _external_heap_start = ABSOLUTE(.);
  } > psram_seg
}

_external_ram_start = ABSOLUTE(ORIGIN(psram_seg));
_external_ram_end = ABSOLUTE(ORIGIN(psram_seg)+LENGTH(psram_seg));
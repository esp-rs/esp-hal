/*
 * This section is required to skip flash rodata sections, because `psram_seg`
 * and `drom_seg` are on the same bus
 */

SECTIONS {
  .ext_ram_dummy (NOLOAD) :
  {
    . = ORIGIN(psram_seg) + (_rodata_reserved_end - _rodata_dummy_start);
    
    /* Prepare the alignment of the section above
     */

    . = ALIGN(0x10000);
  } > psram_seg
}
INSERT BEFORE .external.data;
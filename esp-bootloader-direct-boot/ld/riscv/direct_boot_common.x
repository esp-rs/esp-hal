SECTIONS {
  /* `#[ram(reclaimed)]` is not available with direct boot, these stay empty. They define the
     symbols the runtime reads. */
  .dram2_uninit.bss (NOLOAD) : ALIGN(4)
  {
    _dram2_uninit_bss_start = ABSOLUTE(.);
    *(.dram2_uninit.bss .dram2_uninit.bss.*)
    _dram2_uninit_bss_end = ABSOLUTE(.);
  } > dram2_seg

  .dram2_uninit (NOLOAD) : ALIGN(4)
  {
    *(.dram2_uninit .dram2_uninit.*)
  } > dram2_seg

  .espressif.metadata 0 (INFO) :
  {
    KEEP(*(.espressif.metadata));
  }

  .eh_frame 0 (INFO) :
  {
    KEEP(*(.eh_frame));
  }
}

ASSERT(_esp_bootloader_entry == IROM_BASE + DIRECT_BOOT_MAGIC_OFFSET + 8, "
ERROR(esp-bootloader-direct-boot): the entry stub must directly follow the magic words");

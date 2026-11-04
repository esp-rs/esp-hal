/* The ROM maps flash and jumps to the instruction right after the magic words, so the entry stub
   (`src/riscv/mod.rs`) has to follow them directly. */
ENTRY(_esp_bootloader_entry)
EXTERN(_esp_bootloader_entry)

/* The entry stub copies .data, so riscv-rt must not. */
__sdata = 0;
__edata = 0;
__sidata = 0;

/* Chips without these sections leave their copy-table entries empty. */
PROVIDE(_direct_boot_uncached_data_src = 0);
PROVIDE(_direct_boot_uncached_data_dst = 0);
PROVIDE(_direct_boot_uncached_data_end = 0);
PROVIDE(_direct_boot_rtc_fast_text_src = 0);
PROVIDE(_direct_boot_rtc_fast_text_dst = 0);
PROVIDE(_direct_boot_rtc_fast_text_end = 0);
PROVIDE(_direct_boot_rtc_fast_data_src = 0);
PROVIDE(_direct_boot_rtc_fast_data_dst = 0);
PROVIDE(_direct_boot_rtc_fast_data_end = 0);

SECTIONS {
  .direct_boot.header IROM_BASE + DIRECT_BOOT_MAGIC_OFFSET : AT(DIRECT_BOOT_MAGIC_OFFSET)
  {
    LONG(0xaedb041d)
    LONG(0xaedb041d)
    KEEP(*(.direct_boot.entry))

    /* (source in flash, destination start, destination end) word triples. The stub reads the
       table through the data bus, which is not the instruction bus on every chip. */
    . = ALIGN(4);
    _direct_boot_copy_table_start = ABSOLUTE(.) - IROM_BASE + DROM_BASE;
    LONG(_direct_boot_uncached_data_src) LONG(_direct_boot_uncached_data_dst) LONG(_direct_boot_uncached_data_end)
    LONG(_direct_boot_rwtext_src) LONG(_direct_boot_rwtext_dst) LONG(_direct_boot_rwtext_end)
    LONG(_direct_boot_data_src) LONG(_direct_boot_data_dst) LONG(_direct_boot_data_end)
    LONG(_direct_boot_rtc_fast_text_src) LONG(_direct_boot_rtc_fast_text_dst) LONG(_direct_boot_rtc_fast_text_end)
    LONG(_direct_boot_rtc_fast_data_src) LONG(_direct_boot_rtc_fast_data_dst) LONG(_direct_boot_rtc_fast_data_end)
    _direct_boot_copy_table_end = ABSOLUTE(.) - IROM_BASE + DROM_BASE;
  } > ROTEXT
}

/* Flash offset of the first RAM section image. */
_direct_boot_ram_lma = ALIGN(LOADADDR(.direct_boot.header) + SIZEOF(.direct_boot.header), 4);

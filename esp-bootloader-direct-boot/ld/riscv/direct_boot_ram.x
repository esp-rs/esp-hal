/* Sections that live in SRAM. Their images follow the header in flash, starting at
   `_direct_boot_rwtext_lma`, and the entry stub copies them before jumping to `_start`.

   These output sections come before `.text` in the script, so they claim the input sections that
   esp-hal's `esp_hal_rwtext.x` places in RAM before the `.text` catch-all does. */

SECTIONS {
  .rwtext : AT(_direct_boot_rwtext_lma)
  {
    /* The vector tables are written at runtime, so they stay in front of `_rwtext_start`:
       esp-hal's PMP makes `_rwtext_start`..`_rwtext_end` read-only. */
    _trap_section_origin = .;
    KEEP(*(.trap));
    *(.trap.*);
    *(.rwtext.interrupt .rwtext.interrupt.*)

    . = ALIGN(4);
    _rwtext_start = .;
    *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)
    /* Patched SPI-flash ROM functions from esp-rom-sys. */
    *:esp_rom_spiflash.*(.literal .text .literal.* .text.*)

    INCLUDE "esp_hal_rwtext.x"

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
  } > RWTEXT

  _rwtext_len = _rwtext_end - ORIGIN(RWTEXT);

  /* Where IRAM and DRAM are two views of the same SRAM, skip the DRAM view of `.rwtext`. */
  .data ALIGN(_rwtext_end - IRAM_DRAM_OFFSET, 16) : AT(ALIGN(LOADADDR(.rwtext) + SIZEOF(.rwtext), 4))
  {
    _data_start = ABSOLUTE(.);
    INCLUDE "esp_hal_rwdata.x"
    *(.sdata .sdata.* .sdata2 .sdata2.*);
    *(.data .data.*);
    *(.data1)
    *( .dram1 .dram1.*)
    . = ALIGN(4);
    _data_end = ABSOLUTE(.);
  } > RWDATA

  /* LLD would give a NOLOAD section following `.data` a load address in flash and then report it
     as overlapping the flash images, so NOLOAD sections load where they live. */
  .bss (NOLOAD) : AT(ADDR(.bss)) ALIGN(4)
  {
    _bss_start = ABSOLUTE(.);
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.sbss .sbss.* .bss .bss.*);
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _bss_end = ABSOLUTE(.);
  } > RWDATA

  .noinit (NOLOAD) : AT(ADDR(.noinit)) ALIGN(4)
  {
    *(.noinit .noinit.*)
    *(.uninit .uninit.*)
    . = ALIGN(4);
    _noinit_end = .;
  } > RWDATA

  /* must be the last section in RWDATA */
  .stack (NOLOAD) : AT(ADDR(.stack)) ALIGN(4)
  {
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);
    . = ORIGIN(RWDATA) + LENGTH(RWDATA);
    . = ALIGN(4);
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA
}

_sidata = LOADADDR(.data);

/* esp-riscv-rt treats a stack pointer below this as a stack overflow. */
_dram_data_start = _rwtext_end - IRAM_DRAM_OFFSET;

/* `.rwtext` is copied through its DRAM view. */
_direct_boot_rwtext_src = DROM_BASE + LOADADDR(.rwtext);
_direct_boot_rwtext_dst = ADDR(.rwtext) - IRAM_DRAM_OFFSET;
_direct_boot_rwtext_end = _direct_boot_rwtext_dst + SIZEOF(.rwtext);

_direct_boot_data_src = DROM_BASE + LOADADDR(.data);
_direct_boot_data_dst = ADDR(.data);
_direct_boot_data_end = _direct_boot_data_dst + SIZEOF(.data);

/* Flash offset after the `.data` image. */
_direct_boot_data_end_lma = ALIGN(LOADADDR(.data) + SIZEOF(.data), 4);

/* Sections that live in SRAM. Their images follow the header in flash, starting at
   `_direct_boot_rwtext_lma`, and the entry stub copies them before jumping to `_start`.

   These output sections come before `.text` in the script, so they claim the input sections that
   esp-hal's `esp_hal_rwtext.x` places in RAM before the `.text` catch-all does. */

SECTIONS {
  /* The only RAM code section with an `AT`: the sections up to the end of `.rwtext` continue its
     load addresses, so the entry stub copies them as one range. */
  .trap : AT(_direct_boot_rwtext_lma)
  {
    /* The vector tables are written at runtime, so they stay in front of `_rwtext_start`:
       esp-hal's PMP makes `_rwtext_start`..`_rwtext_end` read-only. */
    _trap_section_origin = .;
    KEEP(*(.trap));
    *(.trap.*);
    *(.rwtext.interrupt .rwtext.interrupt.*)

    . = ALIGN(4);
    _rwtext_start = .;
  } > RWTEXT

  INCLUDE "esp_hal_rwtext_sections.x"

  .rwtext : ALIGN(4)
  {
    *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)

    INCLUDE "esp_hal_rwtext.x"
    INCLUDE "esp_hal_rwtext_radio.x"
    . = ALIGN(4);
    _rwtext_end = .;
  } > RWTEXT

  _rwtext_len = ABSOLUTE(_rwtext_end - ORIGIN(RWTEXT));

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

  /* Fills RWDATA up to `.retention.bss`. */
  .stack (NOLOAD) : AT(ADDR(.stack)) ALIGN(4)
  {
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);
    . = (ORIGIN(RWDATA) + LENGTH(RWDATA) - SIZEOF(.retention.bss)) & ~15;
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA

  /* `#[ram(unstable(retention))]` statics. The top of RAM is inside the range that the retention
     DMA reaches on the ESP32-C3. `dram2_seg` is empty, so the `_dram2_uninit_bss_*` symbols mark
     this section, and the runtime zeroes it. */
  .retention.bss (NOLOAD) : AT(ADDR(.retention.bss)) ALIGN(16)
  {
    _dram2_uninit_bss_start = ABSOLUTE(.);
    *(.retention.bss .retention.bss.*)
    _dram2_uninit_bss_end = ABSOLUTE(.);
  } > RWDATA
}

_sidata = LOADADDR(.data);

/* esp-riscv-rt treats a stack pointer below this as a stack overflow. */
_dram_data_start = _rwtext_end - IRAM_DRAM_OFFSET;

/* The RAM code from `.trap` to the end of `.rwtext` is copied through its DRAM view. */
_direct_boot_rwtext_src = DROM_BASE + LOADADDR(.trap);
_direct_boot_rwtext_dst = ADDR(.trap) - IRAM_DRAM_OFFSET;
_direct_boot_rwtext_end = _rwtext_end - IRAM_DRAM_OFFSET;

ASSERT(LOADADDR(.rwtext) - LOADADDR(.trap) == ADDR(.rwtext) - ADDR(.trap), "
ERROR(esp-bootloader-direct-boot): the load addresses of the RAM code are not contiguous");

_direct_boot_data_src = DROM_BASE + LOADADDR(.data);
_direct_boot_data_dst = ADDR(.data);
_direct_boot_data_end = _direct_boot_data_dst + SIZEOF(.data);

/* Flash offset after the `.data` image. */
_direct_boot_data_end_lma = ALIGN(LOADADDR(.data) + SIZEOF(.data), 4);

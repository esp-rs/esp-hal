/* The ROM maps flash and jumps to the instruction right after the magic words, so the entry stub
   (`src/xtensa.rs`) has to follow them directly. */
ENTRY(_esp_bootloader_entry)
EXTERN(_esp_bootloader_entry)

/* High level exception and interrupt routines, from xtensa-lx-rt's `exception.x`. That file
   places `.vectors` without a load address in flash, so it can't be included. */
PROVIDE(__exception = __default_exception);
PROVIDE(__user_exception = __default_user_exception);
PROVIDE(__double_exception = __default_double_exception);
PROVIDE(__level_1_interrupt = __default_interrupt);
PROVIDE(__level_2_interrupt = __default_interrupt);
PROVIDE(__level_3_interrupt = __default_interrupt);
PROVIDE(__level_4_interrupt = __default_interrupt);
PROVIDE(__level_5_interrupt = __default_interrupt);
PROVIDE(__level_6_interrupt = __default_interrupt);
PROVIDE(__level_7_interrupt = __default_interrupt);

PROVIDE(Timer0 = __default_user_exception);
PROVIDE(Timer1 = __default_user_exception);
PROVIDE(Timer2 = __default_user_exception);
PROVIDE(Timer3 = __default_user_exception);
PROVIDE(Profiling = __default_user_exception);
PROVIDE(NMI = __default_user_exception);
PROVIDE(Software0 = __default_user_exception);
PROVIDE(Software1 = __default_user_exception);

PROVIDE(__naked_user_exception = __default_naked_exception);
PROVIDE(__naked_kernel_exception = __default_naked_exception);
PROVIDE(__naked_double_exception = __default_naked_double_exception);
PROVIDE(__naked_level_2_interrupt = __default_naked_level_2_interrupt);
PROVIDE(__naked_level_3_interrupt = __default_naked_level_3_interrupt);
PROVIDE(__naked_level_4_interrupt = __default_naked_level_4_interrupt);
PROVIDE(__naked_level_5_interrupt = __default_naked_level_5_interrupt);
PROVIDE(__naked_level_6_interrupt = __default_naked_level_6_interrupt);
PROVIDE(__naked_level_7_interrupt = __default_naked_level_7_interrupt);

EXTERN(__default_exception);
EXTERN(__default_double_exception);
EXTERN(__default_interrupt);

EXTERN(__default_naked_exception);
EXTERN(__default_naked_double_exception);
EXTERN(__default_naked_level_2_interrupt);
EXTERN(__default_naked_level_3_interrupt);
EXTERN(__default_naked_level_4_interrupt);
EXTERN(__default_naked_level_5_interrupt);
EXTERN(__default_naked_level_6_interrupt);
EXTERN(__default_naked_level_7_interrupt);

/* Flash layout (load address == flash offset):
     padding | magic, entry stub, copy table | .vectors | .rwtext | .data | RTC fast | RTC slow
     | .text | .rodata
   The RAM sections come before `.text` in this script, so they claim the input sections that
   esp-hal's `esp_hal_rwtext.x` places in RAM before the `.text` catch-all does. */
SECTIONS {
  .direct_boot.header IROM_BASE + DIRECT_BOOT_MAGIC_OFFSET : AT(DIRECT_BOOT_MAGIC_OFFSET)
  {
    LONG(0xaedb041d)
    LONG(0xaedb041d)
    KEEP(*(.direct_boot.entry))

    /* (source in flash, destination start, destination end) word triples, read through the
       data bus. */
    . = ALIGN(4);
    _direct_boot_copy_table_start = ABSOLUTE(.) - IROM_BASE + DROM_BASE;
    LONG(_direct_boot_vectors_src) LONG(_direct_boot_vectors_dst) LONG(_direct_boot_vectors_end)
    LONG(_direct_boot_rwtext_src) LONG(_direct_boot_rwtext_dst) LONG(_direct_boot_rwtext_end)
    LONG(_direct_boot_data_src) LONG(_direct_boot_data_dst) LONG(_direct_boot_data_end)
    LONG(_direct_boot_rtc_fast_text_src) LONG(_direct_boot_rtc_fast_text_dst) LONG(_direct_boot_rtc_fast_text_end)
    LONG(_direct_boot_rtc_fast_data_src) LONG(_direct_boot_rtc_fast_data_dst) LONG(_direct_boot_rtc_fast_data_end)
    LONG(_direct_boot_rtc_slow_text_src) LONG(_direct_boot_rtc_slow_text_dst) LONG(_direct_boot_rtc_slow_text_end)
    LONG(_direct_boot_rtc_slow_data_src) LONG(_direct_boot_rtc_slow_data_dst) LONG(_direct_boot_rtc_slow_data_end)
    _direct_boot_copy_table_end = ABSOLUTE(.) - IROM_BASE + DROM_BASE;
  } > ROTEXT

  /* Each Xtensa vector has a fixed 64 byte slot, see xtensa-lx-rt's `exception.x`. */
  .vectors ORIGIN(vectors_seg) : AT(ALIGN(LOADADDR(.direct_boot.header) + SIZEOF(.direct_boot.header), 4)) SUBALIGN(0x40)
  {
    _init_start = ABSOLUTE(.);
    KEEP(*(.WindowOverflow4.text));
    KEEP(*(.WindowUnderflow4.text));
    KEEP(*(.WindowOverflow8.text));
    KEEP(*(.WindowUnderflow8.text));
    KEEP(*(.WindowOverflow12.text));
    KEEP(*(.WindowUnderflow12.text));
    KEEP(*(.Level2InterruptVector.text));
    KEEP(*(.Level3InterruptVector.text));
    KEEP(*(.Level4InterruptVector.text));
    KEEP(*(.Level5InterruptVector.text));
    KEEP(*(.DebugExceptionVector.text));
    KEEP(*(.NMIExceptionVector.text));
    KEEP(*(.KernelExceptionVector.text));
    KEEP(*(.UserExceptionVector.text));
    /* 0x40 of padding between the UserExceptionVector slot and the DoubleExceptionVector slot */
    . = . + 0x40;
    KEEP(*(.DoubleExceptionVector.text));
    . = ALIGN(0x400);
    _init_end = ABSOLUTE(.);
  } > vectors_seg

  /* xtensa-lx-rt's `Reset` lives here, so the entry stub copies it before jumping to it. */
  .rwtext ORIGIN(RWTEXT) : AT(LOADADDR(.vectors) + SIZEOF(.vectors))
  {
    _rwtext_start = .;
    /* Keep each object's literal pool adjacent to its code (see `.text`). */
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

  /* Skip the DRAM view of `.vectors` and `.rwtext`. */
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

  .bss (NOLOAD) : ALIGN(4)
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

  .noinit (NOLOAD) : ALIGN(4)
  {
    *(.noinit .noinit.*)
    *(.uninit .uninit.*)
    . = ALIGN(4);
    _noinit_end = .;
  } > RWDATA

  /* must be the last section in RWDATA */
  .stack (NOLOAD) : ALIGN(4)
  {
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);
    . = ORIGIN(RWDATA) + LENGTH(RWDATA);
    . = ALIGN(4);
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA

  .rtc_fast.text ORIGIN(rtc_fast_seg) : AT(ALIGN(LOADADDR(.data) + SIZEOF(.data), 4))
  {
    . = ALIGN(4);
    *(.rtc_fast.literal .rtc_fast.text .rtc_fast.literal.* .rtc_fast.text.*)
    . = ALIGN(4);
  } > rtc_fast_seg

  .rtc_fast.data : AT(ALIGN(LOADADDR(.rtc_fast.text) + SIZEOF(.rtc_fast.text), 4))
  {
    . = ALIGN(4);
    _rtc_fast_data_start = ABSOLUTE(.);
    *(.rtc_fast.data .rtc_fast.data.*)
    . = ALIGN(4);
    _rtc_fast_data_end = ABSOLUTE(.);
  } > rtc_fast_seg

  .rtc_fast.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_fast_bss_start = ABSOLUTE(.);
    *(.rtc_fast.bss .rtc_fast.bss.*)
    . = ALIGN(4);
    _rtc_fast_bss_end = ABSOLUTE(.);
  } > rtc_fast_seg

  .rtc_fast.persistent (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_fast_persistent_start = ABSOLUTE(.);
    *(.rtc_fast.persistent .rtc_fast.persistent.*)
    . = ALIGN(4);
    _rtc_fast_persistent_end = ABSOLUTE(.);
  } > rtc_fast_seg

  .rtc_slow.text ORIGIN(rtc_slow_seg) : AT(ALIGN(LOADADDR(.rtc_fast.data) + SIZEOF(.rtc_fast.data), 4))
  {
    . = ALIGN(4);
    *(.rtc_slow.literal .rtc_slow.text .rtc_slow.literal.* .rtc_slow.text.*)
    . = ALIGN(4);
  } > rtc_slow_seg

  .rtc_slow.data : AT(ALIGN(LOADADDR(.rtc_slow.text) + SIZEOF(.rtc_slow.text), 4))
  {
    . = ALIGN(4);
    _rtc_slow_data_start = ABSOLUTE(.);
    *(.rtc_slow.data .rtc_slow.data.*)
    . = ALIGN(4);
    _rtc_slow_data_end = ABSOLUTE(.);
  } > rtc_slow_seg

  .rtc_slow.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_slow_bss_start = ABSOLUTE(.);
    *(.rtc_slow.bss .rtc_slow.bss.*)
    . = ALIGN(4);
    _rtc_slow_bss_end = ABSOLUTE(.);
  } > rtc_slow_seg

  .rtc_slow.persistent (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_slow_persistent_start = ABSOLUTE(.);
    *(.rtc_slow.persistent .rtc_slow.persistent.*)
    . = ALIGN(4);
    _rtc_slow_persistent_end = ABSOLUTE(.);
  } > rtc_slow_seg

  /* Code and read-only data execute in place: each section's address is the base the ROM maps
     flash at plus its flash offset. */
  .text IROM_BASE + ALIGN(LOADADDR(.rtc_slow.data) + SIZEOF(.rtc_slow.data), 16) : AT(ALIGN(LOADADDR(.rtc_slow.data) + SIZEOF(.rtc_slow.data), 16))
  {
    /* The Espressif blobs (esp-wifi-sys / esp-phy / BT) are GCC-compiled and emit per-function
       `.literal.<f>` paired with `.text.<f>` within each object file. Pulling each archive in as a
       single block keeps every function's literal pool within L32R reach of its own code. */
    *libpp.a:(.literal .text .literal.* .text.*)
    *libnet80211.a:(.literal .text .literal.* .text.*)
    *libwpa_supplicant.a:(.literal .text .literal.* .text.*)
    *libmesh.a:(.literal .text .literal.* .text.*)
    *libsmartconfig.a:(.literal .text .literal.* .text.*)
    *libphy.a:(.literal .text .literal.* .text.*)
    *libcoexist.a:(.literal .text .literal.* .text.*)
    *libcore.a:(.literal .text .literal.* .text.*)
    *libregulatory.a:(.literal .text .literal.* .text.*)
    *libespnow.a:(.literal .text .literal.* .text.*)
    *libwapi.a:(.literal .text .literal.* .text.*)
    *libprintf.a:(.literal .text .literal.* .text.*)
    *libbtdm_app.a:(.literal .text .literal.* .text.*)
    *libbtbb.a:(.literal .text .literal.* .text.*)

    /* Bare `.literal` holds the `global_asm!` literal pools of the boot code; keep their
       consumers right behind it. */
    *(.literal)
    *(.text.__post_init)
    *(.text.__pre_init)

    /* Catch-all. A single combined group keeps every literal pool within L32R reach of its code. */
    *(.literal.* .text .text.*)
    . = ALIGN(4);
  } > ROTEXT

  .rodata ALIGN(DROM_BASE + LOADADDR(.text) + SIZEOF(.text), 16) : AT(ADDR(.rodata) - DROM_BASE)
  {
    _rodata_start = ABSOLUTE(.);
    *(.rodata .rodata.*)
    *(.rodata_wlog_*.*)
    . = ALIGN(4);
    _rodata_end = ABSOLUTE(.);
  } > RODATA

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

  /* RAM not used by the data cache. Never initialized. */
  .dcache_reclaimed_uninit (NOLOAD) : ALIGN(4)
  {
    *(.dcache_reclaimed_uninit .dcache_reclaimed_uninit.*)
  } > dcache_reclaimed_seg

  .espressif.metadata 0 (INFO) :
  {
    KEEP(*(.espressif.metadata));
  }

  .eh_frame 0 (INFO) :
  {
    KEEP(*(.eh_frame));
  }
}

/* xtensa-lx-rt reads this, but `__init_data` tells it not to copy `.data`: the stub did. */
_sidata = LOADADDR(.data);

/* `.vectors` and `.rwtext` are copied through their DRAM view. */
_direct_boot_vectors_src = DROM_BASE + LOADADDR(.vectors);
_direct_boot_vectors_dst = ADDR(.vectors) - IRAM_DRAM_OFFSET;
_direct_boot_vectors_end = _direct_boot_vectors_dst + SIZEOF(.vectors);

_direct_boot_rwtext_src = DROM_BASE + LOADADDR(.rwtext);
_direct_boot_rwtext_dst = ADDR(.rwtext) - IRAM_DRAM_OFFSET;
_direct_boot_rwtext_end = _direct_boot_rwtext_dst + SIZEOF(.rwtext);

_direct_boot_data_src = DROM_BASE + LOADADDR(.data);
_direct_boot_data_dst = ADDR(.data);
_direct_boot_data_end = _direct_boot_data_dst + SIZEOF(.data);

_direct_boot_rtc_fast_text_src = DROM_BASE + LOADADDR(.rtc_fast.text);
_direct_boot_rtc_fast_text_dst = ADDR(.rtc_fast.text);
_direct_boot_rtc_fast_text_end = _direct_boot_rtc_fast_text_dst + SIZEOF(.rtc_fast.text);

_direct_boot_rtc_fast_data_src = DROM_BASE + LOADADDR(.rtc_fast.data);
_direct_boot_rtc_fast_data_dst = ADDR(.rtc_fast.data);
_direct_boot_rtc_fast_data_end = _direct_boot_rtc_fast_data_dst + SIZEOF(.rtc_fast.data);

_direct_boot_rtc_slow_text_src = DROM_BASE + LOADADDR(.rtc_slow.text);
_direct_boot_rtc_slow_text_dst = ADDR(.rtc_slow.text);
_direct_boot_rtc_slow_text_end = _direct_boot_rtc_slow_text_dst + SIZEOF(.rtc_slow.text);

_direct_boot_rtc_slow_data_src = DROM_BASE + LOADADDR(.rtc_slow.data);
_direct_boot_rtc_slow_data_dst = ADDR(.rtc_slow.data);
_direct_boot_rtc_slow_data_end = _direct_boot_rtc_slow_data_dst + SIZEOF(.rtc_slow.data);

ASSERT(SIZEOF(.vectors) == 0x400, "
ERROR(esp-bootloader-direct-boot): .vectors must be exactly 0x400 bytes - a vector overflowed its 64-byte slot");
ASSERT(_esp_bootloader_entry == IROM_BASE + DIRECT_BOOT_MAGIC_OFFSET + 8, "
ERROR(esp-bootloader-direct-boot): the entry stub must directly follow the magic words");

/* This memory map assumes the flash cache is on; 
   the blocks used are excluded from the various memory ranges 
   
   see: https://github.com/espressif/esp-idf/blob/5b1189570025ba027f2ff6c2d91f6ffff3809cc2/components/heap/port/esp32s2/memory_layout.c
   for details
   */

/* override entry point */
ENTRY(ESP32Reset)

/* reserved at the start of DRAM */
RESERVE_DRAM = 0x4000;

VECTORS_SIZE = 0x400;

/* reserved at the start of the RTC memories for use by the ULP processor */
RESERVE_RTC_FAST = 0;
RESERVE_RTC_SLOW = 0;

/* define stack size for both cores */
STACK_SIZE = 8k;

/* Specify main memory areas */
MEMORY
{
  vectors_seg ( RX )     : ORIGIN = 0x40020000 + RESERVE_DRAM, len = VECTORS_SIZE /* SRAM0 */
  iram_seg ( RX )        : ORIGIN = 0x40020000 + RESERVE_DRAM + VECTORS_SIZE, len = 192k - RESERVE_DRAM - VECTORS_SIZE /* SRAM0 */

  dram_seg ( RW )        : ORIGIN = 0x3FFB0000 + RESERVE_DRAM + VECTORS_SIZE, len = 192k - RESERVE_DRAM - VECTORS_SIZE

  /* SRAM1; reserved for static ROM usage; can be used for heap.
     Length based on the "_dram0_rtos_reserved_start" symbol from IDF used to delimit the
     ROM data reserved region:
     https://github.com/espressif/esp-idf/blob/bcb34ca7aef4e8d3b97d75ad069b960fb1c17c16/components/heap/port/esp32s2/memory_layout.c#L121-L122
  */
  reserved_for_boot_seg  : ORIGIN = 0x3ffffa10, len = 0x5f0

  /* external flash 
     The 0x20 offset is a convenience for the app binary image generation.
     Flash cache has 64KB pages. The .bin file which is flashed to the chip
     has a 0x18 byte file header, and each segment has a 0x08 byte segment
     header. Setting this offset makes it simple to meet the flash cache MMU's
     constraint that (paddr % 64KB == vaddr % 64KB).)
  */
  irom_seg ( RX )        : ORIGIN = 0x40080020, len = 3M - 0x20
  drom_seg ( R )         : ORIGIN = 0x3F000020, len = 4M - 0x20


  /* RTC fast memory (executable). Persists over deep sleep. Only for core 0 (PRO_CPU) */
  rtc_fast_iram_seg(RWX) : ORIGIN = 0x40070000, len = 8k

  /* RTC fast memory (same block as above), viewed from data bus. Only for core 0 (PRO_CPU) */
  rtc_fast_dram_seg(RW)  : ORIGIN = 0x3ff9e000 + RESERVE_RTC_FAST, len = 8k - RESERVE_RTC_FAST

  /* RTC slow memory (data accessible). Persists over deep sleep. */
  rtc_slow_seg(RW)       : ORIGIN = 0x50000000 + RESERVE_RTC_SLOW, len = 8k - RESERVE_RTC_SLOW

  /* external memory, including data and text */
  psram_seg(RWX)         : ORIGIN =  0x3F500000, len = 0xA80000
}

/* map generic regions to output sections */
INCLUDE "alias.x"

/* esp32 specific regions */
SECTIONS {
  .rtc_fast.text : {
   . = ALIGN(4);
    *(.rtc_fast.literal .rtc_fast.text .rtc_fast.literal.* .rtc_fast.text.*)
  } > rtc_fast_iram_seg AT > RODATA

  /*
    This section is required to skip rtc.text area because rtc_iram_seg and
    rtc_data_seg are reflect the same address space on different buses.
  */
  .rtc_fast.dummy (NOLOAD) :
  {
    _rtc_dummy_start = ABSOLUTE(.); /* needed to make section proper size */
    . = SIZEOF(.rtc_fast.text);
    _rtc_dummy_end = ABSOLUTE(.); /* needed to make section proper size */
  } > rtc_fast_dram_seg
  
  
  .rtc_fast.data :
  {
    . = ALIGN(4);
    _rtc_fast_data_start = ABSOLUTE(.);
    *(.rtc_fast.data .rtc_fast.data.*)
    _rtc_fast_data_end = ABSOLUTE(.);
  } > rtc_fast_dram_seg AT > RODATA

 .rtc_fast.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_fast_bss_start = ABSOLUTE(.);
    *(.rtc_fast.bss .rtc_fast.bss.*)
    _rtc_fast_bss_end = ABSOLUTE(.);
  } > rtc_fast_dram_seg

 .rtc_fast.noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.rtc_fast.noinit .rtc_fast.noinit.*)
  } > rtc_fast_dram_seg


 .rtc_slow.text : {
   . = ALIGN(4);
    *(.rtc_slow.literal .rtc_slow.text .rtc_slow.literal.* .rtc_slow.text.*)
  } > rtc_slow_seg AT > RODATA

  .rtc_slow.data :
  {
    . = ALIGN(4);
    _rtc_slow_data_start = ABSOLUTE(.);
    *(.rtc_slow.data .rtc_slow.data.*)
    _rtc_slow_data_end = ABSOLUTE(.);
  } > rtc_slow_seg AT > RODATA

 .rtc_slow.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_slow_bss_start = ABSOLUTE(.);
    *(.rtc_slow.bss .rtc_slow.bss.*)
    _rtc_slow_bss_end = ABSOLUTE(.);
  } > rtc_slow_seg

 .rtc_slow.noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.rtc_slow.noinit .rtc_slow.noinit.*)
  } > rtc_slow_seg

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

_heap_end = ABSOLUTE(ORIGIN(dram_seg))+LENGTH(dram_seg)+LENGTH(reserved_for_boot_seg) - STACK_SIZE;
_text_heap_end = ABSOLUTE(ORIGIN(iram_seg)+LENGTH(iram_seg));
_external_heap_end = ABSOLUTE(ORIGIN(psram_seg)+LENGTH(psram_seg));

_stack_start_cpu0 = _heap_end;
_stack_end_cpu0 = _stack_start_cpu0 + STACK_SIZE;

EXTERN(DefaultHandler);

INCLUDE "device.x"

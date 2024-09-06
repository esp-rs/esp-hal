/* This memory map assumes the flash cache is on; 
   the blocks used are excluded from the various memory ranges 
   
   see: https://github.com/espressif/esp-idf/blob/5b1189570025ba027f2ff6c2d91f6ffff3809cc2/components/heap/port/esp32s2/memory_layout.c
   for details
   */

/* override entry point */
ENTRY(ESP32Reset)

INCLUDE "memory_extras.x"

VECTORS_SIZE = 0x400;

/* Specify main memory areas */
MEMORY
{
  vectors_seg ( RX )     : ORIGIN = 0x40020000 + RESERVE_CACHES, len = VECTORS_SIZE
  iram_seg ( RX )        : ORIGIN = 0x40020000 + RESERVE_CACHES + VECTORS_SIZE, len = 188k - RESERVE_CACHES - VECTORS_SIZE

  dram_seg ( RW )        : ORIGIN = 0x3FFB0000 + RESERVE_CACHES + VECTORS_SIZE, len = 188k - RESERVE_CACHES - VECTORS_SIZE

  /* memory available after the 2nd stage bootloader is finished */
  dram2_seg ( RW )       : ORIGIN = ORIGIN(dram_seg) + LENGTH(dram_seg), len = 0x3ffffa10 - (ORIGIN(dram_seg) + LENGTH(dram_seg))

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
  rtc_fast_dram_seg(RW)  : ORIGIN = 0x3ff9e000, len = 8k

  /* RTC slow memory (data accessible). Persists over deep sleep. */
  rtc_slow_seg(RW)       : ORIGIN = 0x50000000, len = 8k
}

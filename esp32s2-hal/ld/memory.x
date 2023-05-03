/* This memory map assumes the flash cache is on; 
   the blocks used are excluded from the various memory ranges 
   
   see: https://github.com/espressif/esp-idf/blob/5b1189570025ba027f2ff6c2d91f6ffff3809cc2/components/heap/port/esp32s2/memory_layout.c
   for details
   */

/* override entry point */
ENTRY(ESP32Reset)

/* reserved at the start of DRAM/IRAM */
RESERVE_CACHES = 0x2000;

VECTORS_SIZE = 0x400;

/* reserved at the start of the RTC memories for use by the ULP processor */
RESERVE_RTC_FAST = 0;
RESERVE_RTC_SLOW = 0;

/* define stack size for both cores */
STACK_SIZE = 8k;

/* Specify main memory areas */
MEMORY
{
  vectors_seg ( RX )     : ORIGIN = 0x40020000 + RESERVE_CACHES, len = VECTORS_SIZE /* SRAM0 */
  iram_seg ( RX )        : ORIGIN = 0x40020000 + RESERVE_CACHES + VECTORS_SIZE, len = 192k - RESERVE_CACHES - VECTORS_SIZE /* SRAM0 */

  dram_seg ( RW )        : ORIGIN = 0x3FFB0000 + RESERVE_CACHES + VECTORS_SIZE, len = 192k - RESERVE_CACHES - VECTORS_SIZE

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

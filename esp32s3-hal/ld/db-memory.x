/* override entry point */
ENTRY(ESP32Reset)

/* reserved at the start of DRAM */
RESERVE_DRAM = 0x8000;

/* reserved at the start of the RTC memories for use by the ULP processor */
RESERVE_RTC_FAST = 0;
RESERVE_RTC_SLOW = 0;

/* define stack size for both cores */
STACK_SIZE = 8k;

/* Specify main memory areas */
MEMORY
{
  iram_seg ( RX )        : ORIGIN = 0x40370400 + RESERVE_DRAM, len = 328k - 0x400

  dram_seg ( RW )        : ORIGIN = 0x3FC80000 + RESERVE_DRAM, len = 328k - RESERVE_DRAM
  reserved_for_boot_seg  : ORIGIN = 0x3FFDC200, len = 144k /* ???? SRAM1; reserved for static ROM usage; can be used for heap */

  /* external flash 
     The 0x20 offset is a convenience for the app binary image generation.
     Flash cache has 64KB pages. The .bin file which is flashed to the chip
     has a 0x18 byte file header, and each segment has a 0x08 byte segment
     header. Setting this offset makes it simple to meet the flash cache MMU's
     constraint that (paddr % 64KB == vaddr % 64KB).)
  */
  irom_seg ( RX )        : ORIGIN = 0x42000000, len = 4M
  drom_seg ( R )         : ORIGIN = 0x3C000000, len = 4M


  /* RTC fast memory (executable). Persists over deep sleep. Only for core 0 (PRO_CPU) */
  rtc_fast_iram_seg(RWX) : ORIGIN = 0x600fe000, len = 8k

  /* RTC fast memory (same block as above), viewed from data bus. Only for core 0 (PRO_CPU) */
  rtc_fast_dram_seg(RW)  : ORIGIN = 0x600fe000 + RESERVE_RTC_FAST, len = 8k - RESERVE_RTC_FAST

  /* RTC slow memory (data accessible). Persists over deep sleep. */
  rtc_slow_seg(RW)       : ORIGIN = 0x50000000 + RESERVE_RTC_SLOW, len = 8k - RESERVE_RTC_SLOW

  /* external memory, including data and text */
  psram_seg(RWX)         : ORIGIN =  0x3F500000, len = 0xA80000 /* ??? */
}

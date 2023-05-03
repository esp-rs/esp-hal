/* This memory map assumes the flash cache is on; 
   the blocks used are excluded from the various memory ranges 
   
   see: https://github.com/espressif/esp-idf/blob/5b1189570025ba027f2ff6c2d91f6ffff3809cc2/components/heap/port/esp32/memory_layout.c
   for details
   */

/* override entry point */
ENTRY(ESP32Reset)

INCLUDE "memory_extras.x"

/* Specify main memory areas */
MEMORY
{
  reserved_cache_seg     : ORIGIN = 0x40070000, len = 64k /* SRAM0; reserved for usage as flash cache*/
  vectors_seg ( RX )     : ORIGIN = 0x40080000, len =  1k /* SRAM0 */
  iram_seg ( RX )        : ORIGIN = 0x40080400, len = 128k-0x400 /* SRAM0 */

  reserved_for_rom_seg   : ORIGIN = 0x3FFAE000, len = 8k /* SRAM2; reserved for usage by the ROM */
  dram_seg ( RW )        : ORIGIN = 0x3FFB0000 + RESERVE_DRAM, len = 176k - RESERVE_DRAM /* SRAM2+1; first 64kB used by BT if enable */
  
  /* 
  * The following values come from the heap allocator in esp-idf: https://github.com/espressif/esp-idf/blob/ab63aaa4a24a05904da2862d627f3987ecbeafd0/components/heap/port/esp32/memory_layout.c#L137-L157
  * The segment dram2_seg after the rom data space is not mentioned in the esp32 linker scripts in esp-idf, instead the space after is used as heap space.
  * It seems not all rom data space is reserved, but only "core"/"important" ROM functions that may be called after booting from ROM.
  */
  reserved_rom_data_pro  : ORIGIN = 0X3FFE0000, len = 1088
  reserved_rom_data_app  : ORIGIN = 0X3FFE3F20, len = 1072

  dram2_seg              : ORIGIN = 0x3FFE4350, len = 111k /* the rest of DRAM after the rom data segments in the middle */

  /* external flash 
     The 0x20 offset is a convenience for the app binary image generation.
     Flash cache has 64KB pages. The .bin file which is flashed to the chip
     has a 0x18 byte file header, and each segment has a 0x08 byte segment
     header. Setting this offset makes it simple to meet the flash cache MMU's
     constraint that (paddr % 64KB == vaddr % 64KB).)
  */
  irom_seg ( RX )        : ORIGIN = 0x400D0020, len = 3M - 0x20
  drom_seg ( R )         : ORIGIN = 0x3F400020, len = 4M - 0x20


  /* RTC fast memory (executable). Persists over deep sleep. Only for core 0 (PRO_CPU) */
  rtc_fast_iram_seg(RWX) : ORIGIN = 0x400C0000, len = 8k

  /* RTC fast memory (same block as above), viewed from data bus. Only for core 0 (PRO_CPU) */
  rtc_fast_dram_seg(RW)  : ORIGIN = 0x3FF80000 + RESERVE_RTC_FAST, len = 8k - RESERVE_RTC_FAST

  /* RTC slow memory (data accessible). Persists over deep sleep. */
  rtc_slow_seg(RW)       : ORIGIN = 0x50000000 + RESERVE_RTC_SLOW, len = 8k - RESERVE_RTC_SLOW

  /* external memory, including data and text, 
     4MB is the maximum, if external psram is bigger, paging is required */
  psram_seg(RWX)         : ORIGIN = 0x3F800000, len = 4M
}


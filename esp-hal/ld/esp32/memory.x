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
    Start value taken from: https://github.com/espressif/esp-idf/blob/68b583f30ceb08af9afdaf836a89f4bf17972ac8/components/heap/port/esp32/memory_layout.c#L151

    Checkout esp32.rom.ld for functions used from 0x3FFE_0000 to 0x3FFE_40F0
  */
  dram2_seg : ORIGIN = 0x3FFE4350, len = 0x3FFFFFFF - 0x3FFE4350

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
  rtc_fast_dram_seg(RW)  : ORIGIN = 0x3FF80000, len = 8k

  /* RTC slow memory (data accessible). Persists over deep sleep. */
  rtc_slow_seg(RW)       : ORIGIN = 0x50000000, len = 8k
}


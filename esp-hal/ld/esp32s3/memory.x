/* override entry point */
ENTRY(ESP32Reset)

/* reserved for ICACHE */
RESERVE_ICACHE = 0x8000;

VECTORS_SIZE = 0x400;

/* Specify main memory areas

 40370000 <- IRAM/Icache -> 40378000 <- D/IRAM (I) -> 403E0000
                            3FC88000 <- D/IRAM (D) -> 3FCF0000 <- DRAM/DCache -> 3FD00000

 Startup code uses the IRAM from 0x403B9000 to 0x403E0000, which is not available for static
 memory, but can only be used after app starts.

 D cache use the memory from high address, so when it's configured to 16K/32K, the region
 0x3FCF0000 ~ (3FD00000 - DATA_CACHE_SIZE) should be available. This region is not used as
 static memory, leaving to the heap.
*/
MEMORY
{
  vectors_seg ( RX )     : ORIGIN = 0x40370000 + RESERVE_ICACHE, len = VECTORS_SIZE
  iram_seg ( RX )        : ORIGIN = 0x40370000 + RESERVE_ICACHE + VECTORS_SIZE, len = 328k - VECTORS_SIZE - RESERVE_ICACHE
  dram_seg ( RW )        : ORIGIN = 0x3FC88000 , len = 345856 

  /* memory available after the 2nd stage bootloader is finished */
  dram2_seg ( RW )       : ORIGIN = ORIGIN(dram_seg) + LENGTH(dram_seg), len = 0x3fced710 - (ORIGIN(dram_seg) + LENGTH(dram_seg))

  /* external flash 
     The 0x20 offset is a convenience for the app binary image generation.
     Flash cache has 64KB pages. The .bin file which is flashed to the chip
     has a 0x18 byte file header, and each segment has a 0x08 byte segment
     header. Setting this offset makes it simple to meet the flash cache MMU's
     constraint that (paddr % 64KB == vaddr % 64KB).)
  */
  irom_seg ( RX )        : ORIGIN = 0x42000020, len = 4M - 0x20
  drom_seg ( R )         : ORIGIN = 0x3C000020, len = 4M - 0x20


  /* RTC fast memory (executable). Persists over deep sleep. Only for core 0 (PRO_CPU) */
  rtc_fast_seg(RWX) : ORIGIN = 0x600fe000, len = 8k

  /* RTC slow memory (data accessible). Persists over deep sleep. */
  rtc_slow_seg(RW)       : ORIGIN = 0x50000000, len = 8k
}

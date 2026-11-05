/* The ROM looks for the magic at flash offset 0x400, jumps to 0x42000408 and maps the first 4 MB
   of flash at IROM_BASE and DROM_BASE. */
DIRECT_BOOT_MAGIC_OFFSET = 0x400;
IROM_BASE = 0x42000000;
DROM_BASE = 0x3C000000;

/* IRAM 0x40378000 and DRAM 0x3FC88000 address the same SRAM word. */
IRAM_DRAM_OFFSET = 0x40378000 - 0x3FC88000;

/*
 40370000 <- IRAM/Icache -> 40378000 <- D/IRAM (I) -> 403E0000
                            3FC88000 <- D/IRAM (D) -> 3FCF0000 <- DRAM/DCache -> 3FD00000

 The ICache size the ROM boots with is not known, so the vectors go right after the largest
 (32 KB) ICache configuration.
*/
MEMORY
{
  vectors_seg ( RX )     : ORIGIN = 0x40378000, len = 0x400
  iram_seg ( RX )        : ORIGIN = 0x40378400, len = 0x403C2000 - 0x40378400

  /* Ends at SOC_ROM_STACK_START, the top of the PRO CPU's ROM stack. */
  dram_seg ( RW )        : ORIGIN = 0x3FC88000, len = 0x3FCEB710 - 0x3FC88000
  /* `#[ram(reclaimed)]` is not available with direct boot. */
  dram2_seg ( RW )       : ORIGIN = 0x3FCEB710, len = 0

  dcache_reclaimed_seg ( RW ) : ORIGIN = 0x3FCF0000, len = 0x3FD00000 - 0x3FCF0000 - ESP_HAL_DATA_CACHE_SIZE

  irom_seg ( RX )        : ORIGIN = 0x42000000, len = 4M
  drom_seg ( R )         : ORIGIN = 0x3C000000, len = 4M

  /* RTC fast memory (executable). Persists over deep sleep. Only for core 0 (PRO_CPU) */
  rtc_fast_seg ( RWX )   : ORIGIN = 0x600fe000, len = 8k

  /* RTC slow memory (data accessible). Persists over deep sleep. */
  rtc_slow_seg ( RW )    : ORIGIN = 0x50000000, len = 8k
}

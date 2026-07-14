/* The 768 KB L2MEM is shared between L2 cache (low end) and L2 RAM
   (rest). The 2nd-stage bootloader programs CACHE.L2_CACHE_CACHESIZE_CONF
   to one of four splits; the L2 RAM ORIGIN below must match. Top is
   fixed at 0x4FFAE000 to leave the v3.x ROM BSS/stack region (top
   ~72 KB of L2MEM) untouched.

   CONFIG numbers follow TRM Figure 7.3-2 (HP L2MEM Address Layout):

     CONFIG   ESP_HAL_CONFIG_L2_CACHE_SIZE   L2 cache    L2 RAM ORIGIN
     ------   ----------------------------   --------    -------------
       0      "512KB"                        512 KB      0x4FF80000
       1      "256KB"                        256 KB      0x4FF40000
       2      "128KB"  (IDF default)         128 KB      0x4FF20000
       3      "0KB"                            0 KB      0x4FF00000
*/

#IF ESP_HAL_CONFIG_L2_CACHE_SIZE == "512KB"
RESERVED_L2_CACHE = 0x80000;
#ELSE IF ESP_HAL_CONFIG_L2_CACHE_SIZE == "256KB"
RESERVED_L2_CACHE = 0x40000;
#ELSE IF ESP_HAL_CONFIG_L2_CACHE_SIZE == "128KB"
RESERVED_L2_CACHE = 0x20000;
#ELSE IF ESP_HAL_CONFIG_L2_CACHE_SIZE == "0KB"
RESERVED_L2_CACHE = 0;
#ENDIF

MEMORY
{
    RAM : ORIGIN = 0x4FF00000 + RESERVED_L2_CACHE, LENGTH = 0x4FFAE000 - RESERVED_L2_CACHE - 0x4FF00000

    /* External flash (XIP via cache); +0x20 skips the IDF app image header. */
    ROM : ORIGIN = 0x40000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* LP SRAM (32 KB, persists over deep sleep).

       The first 0x100 bytes are reserved: on ESP32-P4 rev 3.0 (ECO5) the
       deep-sleep wake reset vector is redirected to LP-RAM base 0x50108000
       (LP_CLKRST hpcore0_stat_vector_sel = 0) to run the MSPI-crash-after-
       power-up workaround stub before jumping to HP ROM. The stub is copied
       there at deep-sleep entry, so nothing else may be linked into it. */
    RTC_FAST : ORIGIN = 0x50108000 + 0x100, LENGTH = 32K - 0x100
}

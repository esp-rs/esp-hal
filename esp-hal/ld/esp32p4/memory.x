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

MEMORY
{
    /* CONFIG 0 */
#IF ESP_HAL_CONFIG_L2_CACHE_SIZE_512KB
    RAM : ORIGIN = 0x4FF80000, LENGTH = 0x4FFAE000 - 0x4FF80000
#ENDIF
    /* CONFIG 1 */
#IF ESP_HAL_CONFIG_L2_CACHE_SIZE_256KB
    RAM : ORIGIN = 0x4FF40000, LENGTH = 0x4FFAE000 - 0x4FF40000
#ENDIF
    /* CONFIG 2 */
#IF ESP_HAL_CONFIG_L2_CACHE_SIZE_128KB
    RAM : ORIGIN = 0x4FF20000, LENGTH = 0x4FFAE000 - 0x4FF20000
#ENDIF
    /* CONFIG 3 */
#IF ESP_HAL_CONFIG_L2_CACHE_SIZE_0KB
    RAM : ORIGIN = 0x4FF00000, LENGTH = 0x4FFAE000 - 0x4FF00000
#ENDIF

    /* External flash (XIP via cache); +0x20 skips the IDF app image header. */
    ROM : ORIGIN = 0x40000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* LP SRAM (32 KB, persists over deep sleep). */
    RTC_FAST : ORIGIN = 0x50108000, LENGTH = 32K
}

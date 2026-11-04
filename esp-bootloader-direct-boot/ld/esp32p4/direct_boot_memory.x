/* The ROM looks for the magic at flash offset 0 and maps flash at 0x40000000 for both
   instruction and data access. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x40000000;
DROM_BASE = 0x40000000;
IRAM_DRAM_OFFSET = 0;

/* The 768 KB L2MEM is shared between L2 cache (low end) and L2 RAM (rest). The entry stub
   programs the split ESP_HAL_CONFIG_L2_CACHE_SIZE selects, see `src/riscv/mod.rs`. Top is fixed
   at 0x4FFAE000 to leave the v3.x ROM BSS/stack region (top ~72 KB of L2MEM) untouched. */
ASSERT(ESP_HAL_L2_CACHE_SIZE != 0, "
ERROR(esp-bootloader-direct-boot): ESP_HAL_CONFIG_L2_CACHE_SIZE=0KB is not supported");

/* `cache_size_t` of the ROM's `Cache_Set_L2_Cache_Mode`. */
_direct_boot_l2_cache_size = ESP_HAL_L2_CACHE_SIZE == 0x80000 ? 11 : ESP_HAL_L2_CACHE_SIZE == 0x40000 ? 10 : 9;

MEMORY
{
    RAM : ORIGIN = 0x4FF00000 + ESP_HAL_L2_CACHE_SIZE, LENGTH = 0xADFC0 - ESP_HAL_L2_CACHE_SIZE

    /* L1-bypass alias of RAM. The address is the RAM address + 0x40000000. */
    UNCACHE (RW) : ORIGIN = 0x8FF00000 + ESP_HAL_L2_CACHE_SIZE, LENGTH = 0xADFC0 - ESP_HAL_L2_CACHE_SIZE

    /* Used by the ROM during boot, free afterwards. Upper bound is __stack_app = 0x4ffbefc0. */
    dram2_seg ( RW ) : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4ffbefc0 - (ORIGIN(RAM) + LENGTH(RAM))

    /* The MMU maps up to 1024 pages of 64 KB of NOR flash, 64 MB from 0x40000000. */
    ROM : ORIGIN = 0x40000000, LENGTH = 0x4000000

    /* LP SRAM (32 KB, persists over deep sleep).

       The first 0x100 bytes are reserved: on ESP32-P4 rev 3.0 (ECO5) the deep-sleep wake reset
       vector is redirected to LP-RAM base 0x50108000 to run the MSPI-crash-after-power-up
       workaround stub before jumping to HP ROM. */
    RTC_FAST : ORIGIN = 0x50108000 + 0x100, LENGTH = 32K - 0x100
}

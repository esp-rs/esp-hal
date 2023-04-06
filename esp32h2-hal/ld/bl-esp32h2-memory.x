MEMORY
{
     /* MEMORY_MAP = [
        [0x00000000, 0x00010000, "PADDING"],
        [0x42800000, 0x43000000, "DROM"],
        [0x40800000, 0x40850000, "RAM"],
        [0x40800000, 0x40850000, "BYTE_ACCESSIBLE"],
        [0x4001С400, 0x40020000, "DROM_MASK"],
        [0x40000000, 0x4001С400, "ROM_MASK"],
        [0x42000000, 0x42800000, "ROM"],
        [0x40800000, 0x40850000, "RAM"],
        [0x50000000, 0x50001000, "RTC_RAM"],
        [0x50000000, 0x50001000, "RTC_RAM"],
        [0x600FE000, 0x60100000, "MEM_INTERNAL2"],
    ] */


    /* 320K of on soc RAM, 16K reserved for cache */
    ICACHE : ORIGIN = 0x40800000,  LENGTH = 16K
    /* Instruction and Data RAM */
    RAM : ORIGIN = 0x40800000 + 16K, LENGTH = 320K - 16K

    /* External flash */
    /* Instruction and Data ROM */
    ROM : ORIGIN =   0x42000000, LENGTH = 0x400000

    /* RTC fast memory (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50000000, LENGTH = 16K /*- ESP_BOOTLOADER_RESERVE_RTC*/
}

REGION_ALIAS("ROTEXT", ROM);
REGION_ALIAS("RODATA", ROM);

REGION_ALIAS("RWTEXT", RAM);
REGION_ALIAS("RWDATA", RAM);

REGION_ALIAS("RTC_FAST_RWTEXT", RTC_FAST);
REGION_ALIAS("RTC_FAST_RWDATA", RTC_FAST);

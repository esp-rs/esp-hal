/* The ROM looks for the magic at flash offset 0 and maps flash at IROM_BASE and DROM_BASE. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x42000000;
DROM_BASE = 0x3C000000;

/* IRAM 0x40380000 and DRAM 0x3FC80000 address the same SRAM word. */
IRAM_DRAM_OFFSET = 0x40380000 - 0x3FC80000;

MEMORY
{
    ICACHE : ORIGIN = 0x4037C000, LENGTH = 0x4000
    IRAM : ORIGIN = 0x4037C000 + 0x4000, LENGTH = 313K - 0x4000
    DRAM : ORIGIN = 0x3FC80000, LENGTH = 313K

    /* used by the ROM during boot, free afterwards */
    dram2_seg ( RW ) : ORIGIN = ORIGIN(DRAM) + LENGTH(DRAM), len = 0x3fcde710 - (ORIGIN(DRAM) + LENGTH(DRAM))

    IROM : ORIGIN = 0x42000000, LENGTH = 0x800000
    DROM : ORIGIN = 0x3C000000, LENGTH = 0x800000

    /* RTC fast memory (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50000000, LENGTH = 0x2000
}

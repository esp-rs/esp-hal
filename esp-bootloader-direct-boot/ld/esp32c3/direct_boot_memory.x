/* The ROM looks for the magic at flash offset 0 and maps flash at IROM_BASE and DROM_BASE. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x42000000;
DROM_BASE = 0x3C000000;

/* IRAM 0x40380000 and DRAM 0x3FC80000 address the same SRAM word. */
IRAM_DRAM_OFFSET = 0x40380000 - 0x3FC80000;

/* RAM ends at SOC_ROM_STACK_START. */
MEMORY
{
    ICACHE : ORIGIN = 0x4037C000, LENGTH = 0x4000
    IRAM : ORIGIN = 0x4037C000 + 0x4000, LENGTH = 0x3FCDE710 - 0x3FC80000 - 0x4000
    DRAM : ORIGIN = 0x3FC80000, LENGTH = 0x3FCDE710 - 0x3FC80000

    /* `#[ram(reclaimed)]` is not available with direct boot. */
    dram2_seg ( RW ) : ORIGIN = 0x3FCDE710, len = 0

    IROM : ORIGIN = 0x42000000, LENGTH = 0x800000
    DROM : ORIGIN = 0x3C000000, LENGTH = 0x800000

    /* RTC fast memory (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50000000, LENGTH = 0x2000
}

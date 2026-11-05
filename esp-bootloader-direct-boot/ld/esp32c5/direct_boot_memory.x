/* The ROM looks for the magic at flash offset 0 and maps flash at 0x42000000 for both
   instruction and data access. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x42000000;
DROM_BASE = 0x42000000;
IRAM_DRAM_OFFSET = 0;

/* RAM ends at SOC_ROM_STACK_START. */
MEMORY
{
    RAM : ORIGIN = 0x40800000, LENGTH = 0x4085E5A0 - 0x40800000

    /* `#[ram(reclaimed)]` is not available with direct boot. */
    dram2_seg ( RW ) : ORIGIN = 0x4085E5A0, len = 0

    ROM : ORIGIN = 0x42000000, LENGTH = 0x1000000

    /* RTC fast memory (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50000000, LENGTH = 16K
}

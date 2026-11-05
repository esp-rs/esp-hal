/* The ROM looks for the magic at flash offset 0 and maps flash at 0x42000000 for both
   instruction and data access. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x42000000;
DROM_BASE = 0x42000000;
IRAM_DRAM_OFFSET = 0;

/* RAM ends at SOC_ROM_STACK_START, the top of the first core's ROM stack. */
MEMORY
{
    /* Shared instruction and data memory, starts at SOC_IRAM_LOW. */
    RAM : ORIGIN = 0x40810000, LENGTH = 0x4085D350 - 0x40810000

    /* `#[ram(reclaimed)]` is not available with direct boot. */
    dram2_seg ( RW ) : ORIGIN = 0x4085D350, len = 0

    ROM : ORIGIN = 0x42000000, LENGTH = 0x1000000
}

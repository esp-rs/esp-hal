/* The ROM looks for the magic at flash offset 0 and maps flash at 0x42000000 for both
   instruction and data access. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x42000000;
DROM_BASE = 0x42000000;
IRAM_DRAM_OFFSET = 0;

MEMORY
{
    /* Shared instruction and data memory, starts at SOC_IRAM_LOW. */
    RAM : ORIGIN = 0x40810000, LENGTH = 0x3F350

    /* Used by the ROM during boot, free afterwards. Upper bound is SOC_ROM_STACK_START. */
    dram2_seg ( RW ) : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4085d350 - (ORIGIN(RAM) + LENGTH(RAM))

    ROM : ORIGIN = 0x42000000, LENGTH = 0x1000000
}

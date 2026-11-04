/* The ROM looks for the magic at flash offset 0 and maps flash at 0x42000000 for both
   instruction and data access. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x42000000;
DROM_BASE = 0x42000000;
IRAM_DRAM_OFFSET = 0;

MEMORY
{
    RAM : ORIGIN = 0x40800000, LENGTH = 0x3EA70

    /* used by the ROM during boot, free afterwards */
    dram2_seg ( RW ) : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4084EA70 - (ORIGIN(RAM) + LENGTH(RAM))

    ROM : ORIGIN = 0x42000000, LENGTH = 0x1000000
}

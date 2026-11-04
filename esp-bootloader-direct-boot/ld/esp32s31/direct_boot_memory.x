/* The ROM looks for the magic at flash offset 0 and maps flash at 0x40000000 for both
   instruction and data access. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x40000000;
DROM_BASE = 0x40000000;
IRAM_DRAM_OFFSET = 0;

MEMORY
{
    /* HP SRAM, shared instruction and data memory. */
    RAM : ORIGIN = 0x2F000000, LENGTH = 0x6CFB0

    /* Used by the ROM during boot, free afterwards. Upper bound is SOC_ROM_STACK_START. */
    dram2_seg ( RW ) : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x2f07cfb0 - (ORIGIN(RAM) + LENGTH(RAM))

    ROM : ORIGIN = 0x40000000, LENGTH = 0x4000000

    /* LP SRAM (32 KB, persists over deep sleep). */
    RTC_FAST : ORIGIN = 0x2E000000, LENGTH = 0x8000
}

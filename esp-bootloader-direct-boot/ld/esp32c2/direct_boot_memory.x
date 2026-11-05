/* The ROM looks for the magic at flash offset 0 and maps flash at IROM_BASE and DROM_BASE. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x42000000;
DROM_BASE = 0x3C000000;

/* IRAM 0x40380000 and DRAM 0x3FCA0000 address the same SRAM word. */
IRAM_DRAM_OFFSET = 0x40380000 - 0x3FCA0000;

/* RAM ends at SOC_ROM_STACK_START, where the rev1.0 ROM's `.bss`/`.data` starts. The rev2.0 ROM
   starts it at 0x3FCDEA40, with data used only during boot, which is free after the handoff. */
MEMORY
{
    ICACHE : ORIGIN = 0x4037C000, LENGTH = 16K
    IRAM : ORIGIN = 0x4037C000 + LENGTH(ICACHE), LENGTH = 0x3FCDEB70 - 0x3FCA0000
    DRAM : ORIGIN = 0x3FCA0000, LENGTH = 0x3FCDEB70 - 0x3FCA0000

    /* `#[ram(reclaimed)]` is not available with direct boot. */
    dram2_seg ( RW ) : ORIGIN = 0x3FCDEB70, len = 0

    IROM : ORIGIN = 0x42000000, LENGTH = 0x400000
    DROM : ORIGIN = 0x3C000000, LENGTH = 0x400000
}

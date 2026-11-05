/* The ROM looks for the magic at flash offset 0 and maps flash at 0x40000000 for both
   instruction and data access. */
DIRECT_BOOT_MAGIC_OFFSET = 0;
IROM_BASE = 0x40000000;
DROM_BASE = 0x40000000;
IRAM_DRAM_OFFSET = 0;

MEMORY
{
    /* HP SRAM, shared instruction and data memory. Ends at the top of the first core's ROM
       stack: ESP-IDF v6.1 puts it at 0x2F07CFB0, the v0.0 ROM at 0x2F07C600, the lower one is
       used. */
    RAM : ORIGIN = 0x2F000000, LENGTH = 0x2F07C600 - 0x2F000000

    /* `#[ram(reclaimed)]` is not available with direct boot. */
    dram2_seg ( RW ) : ORIGIN = 0x2F07C600, len = 0

    ROM : ORIGIN = 0x40000000, LENGTH = 0x4000000

    /* LP SRAM (32 KB, persists over deep sleep). */
    RTC_FAST : ORIGIN = 0x2E000000, LENGTH = 0x8000
}

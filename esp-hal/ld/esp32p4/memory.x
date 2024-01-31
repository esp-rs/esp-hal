MEMORY
{
    /* MEMORY_MAP = [
        [0x00000000, 0x00010000, "PADDING"],
        [0x40000000, 0x4C000000, "DROM"],
        [0x4FF00000, 0x4FFA0000, "DRAM"],
        [0x4FF00000, 0x4FFA0000, "BYTE_ACCESSIBLE"],
        [0x4FC00000, 0x4FC20000, "DROM_MASK"],
        [0x4FC00000, 0x4FC20000, "IROM_MASK"],
        [0x40000000, 0x4C000000, "IROM"],
        [0x4FF00000, 0x4FFA0000, "IRAM"],
        [0x50108000, 0x50110000, "RTC_IRAM"],
        [0x50108000, 0x50110000, "RTC_DRAM"],
        [0x600FE000, 0x60100000, "MEM_INTERNAL2"],
    ] */

    /* 768K of on soc RAM */
    RAM : ORIGIN = 0x4FF00000, LENGTH = 0xC0000

    /* External flash */
    /* Instruction and Data ROM */
    ROM : ORIGIN = 0x40000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* RTC fast memory (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50108000, LENGTH = 32K /*- ESP_BOOTLOADER_RESERVE_RTC*/
}

MEMORY
{
     /* MEMORY_MAP = [
        [0x42000000, 0x46000000, "DROM"],
        [0x40800000, 0x4084ea70, "DRAM"],
        [0x40800000, 0x4084ea70, "BYTE_ACCESSIBLE"],
        [0x42000000, 0x46000000, "IROM"],
        [0x40800000, 0x4084ea70, "IRAM"],
        [0x50000000, 0x50004000, "RTC_IRAM"],
        [0x50000000, 0x50004000, "RTC_DRAM"],
    ] */

    /* 320K of on-chip SRAM, portion reserved for cache */
    /* Instruction and Data RAM
    0x4083ea70 = 2nd stage bootloader iram_loader_seg start address
    see https://github.com/espressif/esp-idf/blob/master/components/esp_system/ld/esp32c61/memory.ld.in
    */
    RAM : ORIGIN = 0x40800000, LENGTH = 0x3EA70

    /* memory available after the 2nd stage bootloader is finished */
    dram2_seg ( RW )       : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4084EA70 - (ORIGIN(RAM) + LENGTH(RAM))

    /* External flash
     The 0x20 offset is a convenience for the app binary image generation.
     Flash cache has 64KB pages. The .bin file which is flashed to the chip
     has a 0x18 byte file header, and each segment has a 0x08 byte segment
     header. Setting this offset makes it simple to meet the flash cache MMU's
     constraint that (paddr % 64KB == vaddr % 64KB).)
    */

    /* Instruction and Data ROM */
    ROM : ORIGIN =   0x42000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* RTC fast memory (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50000000, LENGTH = 16K /*- ESP_BOOTLOADER_RESERVE_RTC*/
}

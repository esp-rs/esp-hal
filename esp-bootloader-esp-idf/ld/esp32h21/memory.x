MEMORY
{
    /* From esptool, see https://github.com/espressif/esptool/blob/dfd2340/esptool/targets/esp32h21.py#L38
     MEMORY_MAP = [
        [0x00000000, 0x00010000, "PADDING"],
        [0x42000000, 0x43000000, "DROM"],
        [0x40800000, 0x40850000, "DRAM"],
        [0x40800000, 0x40850000, "BYTE_ACCESSIBLE"],
        [0x40000000, 0x40020000, "DROM_MASK"],
        [0x40000000, 0x40020000, "IROM_MASK"],
        [0x42000000, 0x43000000, "IROM"],
        [0x40800000, 0x40850000, "IRAM"],
        [0x50000000, 0x50001000, "RTC_IRAM"],
        [0x50000000, 0x50001000, "RTC_DRAM"],
        [0x40800000, 0x40850000, "MEM_INTERNAL"],
    ] */


    /* 320K of on soc RAM, shared instruction and data memory.
       0x4083BA78 = 2nd stage bootloader iram_loader_seg start address
       see https://github.com/espressif/esp-idf/blob/ad20ded62a2/components/esp_system/ld/esp32h21/memory.ld.in#L20
    */
    RAM : ORIGIN = 0x40800000, LENGTH = 0x3BA78

    /* memory available after the 2nd stage bootloader is finished.
       Upper bound is the start of the ROM .bss and .data that the ROM keeps using,
       see https://github.com/espressif/esp-idf/blob/ad20ded62a2/components/bootloader/subproject/main/ld/esp32h21/bootloader.memory.ld.in#L12-L16
    */
    dram2_seg ( RW )       : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4084fee0 - (ORIGIN(RAM) + LENGTH(RAM))

    /* External flash

     The 0x20 offset is a convenience for the app binary image generation.
     Flash cache has 64KB pages. The .bin file which is flashed to the chip
     has a 0x18 byte file header, and each segment has a 0x08 byte segment
     header. Setting this offset makes it simple to meet the flash cache MMU's
     constraint that (paddr % 64KB == vaddr % 64KB).)
     See <https://github.com/espressif/esp-idf/blob/ad20ded62a2/components/esp_system/ld/esp32h21/memory.ld.in#L40>.
    */

    /* Instruction and Data ROM */
    ROM : ORIGIN =   0x42000000 + 0x20, LENGTH = 0x1000000 - 0x20

    /* LP RAM (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50000000, LENGTH = 4K
}

MEMORY
{
     /* MEMORY_MAP = [
        [0x2F000000, 0x2F080000, "DRAM/IRAM"],
        [0x40000000, 0x50000000, "Flash (256 MB, XIP via cache)"],
        [0x2E000000, 0x2E008000, "LP RAM (RTC)"],
    ] */

    /* HP SRAM shared instruction/data memory.
       0x2F06CFB0 = 2nd stage bootloader iram_loader_seg start address.
       Source: esp-idf components/bootloader/subproject/main/ld/esp32s31/bootloader.memory.ld.in
    */
    RAM : ORIGIN = 0x2F000000, LENGTH = 0x6CFB0

    /* Memory available after the 2nd stage bootloader finishes.
       Upper bound is SOC_ROM_STACK_START = 0x2f07cfb0 (pro cpu ROM stack).
       Source: esp-idf components/soc/esp32s31/include/soc/soc.h
    */
    dram2_seg ( RW )       : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x2f07cfb0 - (ORIGIN(RAM) + LENGTH(RAM))

    /* External flash (XIP via cache).
       The 0x20 offset is a convenience for app binary image generation.
       Flash cache has 64 KB pages. The .bin file has a 0x18-byte file header
       and each segment has a 0x08-byte segment header. This offset makes it
       simple to meet the flash cache MMU constraint (paddr % 64KB == vaddr % 64KB).
       Source: esp-idf components/esp_system/ld/esp32s31/memory.ld.in
    */
    ROM : ORIGIN = 0x40000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* LP SRAM (32 KB, persists over deep sleep).
       Source: esp-idf components/esp_system/ld/esp32s31/memory.ld.in (lp_ram_seg)
    */
    RTC_FAST : ORIGIN = 0x2E000000, LENGTH = 0x8000
}

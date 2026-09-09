MEMORY
{
     /* MEMORY_MAP = [
        [0x00000000, 0x00010000, "PADDING"],
        [0x42000000, 0x44000000, "DROM"],
        [0x40810000, 0x40860000, "DRAM"],
        [0x40810000, 0x40860000, "BYTE_ACCESSIBLE"],
        [0x40000000, 0x40050000, "DROM_MASK"],
        [0x40000000, 0x40050000, "IROM_MASK"],
        [0x42000000, 0x44000000, "IROM"],
        [0x40810000, 0x40860000, "IRAM"],
        [0x40860000, 0x40868000, "ICACHE1"],
        [0x600FE000, 0x60100000, "MEM_INTERNAL2"],
    ] */

    /* 320K of on soc RAM, shared instruction and data memory. It starts at
       SOC_IRAM_LOW, see esp-idf components/soc/esp32h4/include/soc/soc.h
       0x4084f350 = 2nd stage bootloader iram_loader_seg start address
       see esp-idf components/esp_system/ld/esp32h4/memory.ld.in
    */
    RAM : ORIGIN = 0x40810000, LENGTH = 0x3F350

    /* memory available after the 2nd stage bootloader is finished.
       Upper bound is SOC_ROM_STACK_START = 0x4085d350 (pro cpu ROM stack).
       see esp-idf components/soc/esp32h4/include/soc/soc.h
    */
    dram2_seg ( RW )       : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4085d350 - (ORIGIN(RAM) + LENGTH(RAM))

    /* External flash

     The 0x20 offset is a convenience for the app binary image generation.
     Flash cache has 64KB pages. The .bin file which is flashed to the chip
     has a 0x18 byte file header, and each segment has a 0x08 byte segment
     header. Setting this offset makes it simple to meet the flash cache MMU's
     constraint that (paddr % 64KB == vaddr % 64KB).)
    */

    /* Instruction and Data ROM */
    ROM : ORIGIN =   0x42000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* The LP (RTC) memory at 0x50000000..0x50004000 is not usable by
       applications: SOC_RTC_FAST_MEM_SUPPORTED is not defined for this chip
       (esp-idf soc_caps.h, IDF-12313), so the second stage bootloader rejects
       any segment loaded there ("bad load address range", see
       components/esp_image_verify/src/esp_image_format.c).
    */
}

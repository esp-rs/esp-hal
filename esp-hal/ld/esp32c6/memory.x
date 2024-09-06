MEMORY
{
     /* MEMORY_MAP = [
        [0x00000000, 0x00010000, "PADDING"],
        [0x42800000, 0x43000000, "DROM"],
        [0x40800000, 0x40880000, "RAM"],
        [0x40800000, 0x40880000, "BYTE_ACCESSIBLE"],
        [0x4004AC00, 0x40050000, "DROM_MASK"],
        [0x40000000, 0x4004AC00, "ROM_MASK"],
        [0x42000000, 0x42800000, "ROM"],
        [0x40800000, 0x40880000, "RAM"],
        [0x50000000, 0x50004000, "RTC_RAM"],
        [0x50000000, 0x50004000, "RTC_RAM"],
        [0x600FE000, 0x60100000, "MEM_INTERNAL2"],
    ] */

    /* 512K of on soc RAM, 32K reserved for cache */
    /* Instruction and Data RAM 
    0x4086E610 = 2nd stage bootloader iram_loader_seg start address
    see https://github.com/espressif/esp-idf/blob/03414a15508036c8fc0f51642aed7a264e9527df/components/esp_system/ld/esp32c6/memory.ld.in#L26
    */
    RAM : ORIGIN = 0x40800000 , LENGTH = 0x6E610

    /* memory available after the 2nd stage bootloader is finished */
    dram2_seg ( RW )       : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4087e610 - (ORIGIN(RAM) + LENGTH(RAM))

    /* External flash */
    /* Instruction and Data ROM */
    ROM : ORIGIN =   0x42000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* RTC fast memory (executable). Persists over deep sleep. */
    RTC_FAST : ORIGIN = 0x50000000, LENGTH = 16K /*- ESP_BOOTLOADER_RESERVE_RTC*/
}

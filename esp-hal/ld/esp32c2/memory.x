MEMORY
{
    /*
        https://github.com/espressif/esptool/blob/10828527038d143e049790d330ac4de76ce987d6/esptool/targets/esp32c2.py#L53-L62
        MEMORY_MAP = [[0x00000000, 0x00010000, "PADDING"],
                      [0x3C000000, 0x3C400000, "DROM"],
                      [0x3FCA0000, 0x3FCE0000, "DRAM"],
                      [0x3FC88000, 0x3FD00000, "BYTE_ACCESSIBLE"],
                      [0x3FF00000, 0x3FF50000, "DROM_MASK"],
                      [0x40000000, 0x40090000, "IROM_MASK"],
                      [0x42000000, 0x42400000, "IROM"],
                      [0x4037C000, 0x403C0000, "IRAM"]]
    */

    ICACHE : ORIGIN = 0x4037C000, LENGTH = 16K
    /* Instruction RAM */
    IRAM : ORIGIN = 0x4037C000 + LENGTH(ICACHE), LENGTH = 186k
    /* Data RAM */
    DRAM : ORIGIN = 0x3FCA0000, LENGTH = 186k

    /* memory available after the 2nd stage bootloader is finished */
    dram2_seg ( RW )       : ORIGIN = ORIGIN(DRAM) + LENGTH(DRAM), len = 0x3fcdeb70 - (ORIGIN(DRAM) + LENGTH(DRAM))

    /* External flash */
    /* Instruction ROM */
    IROM : ORIGIN =   0x42000000 + 0x20, LENGTH = 0x200000 - 0x20
    /* Data ROM */
    DROM : ORIGIN = 0x3C000000 + 0x20, LENGTH = 0x200000 - 0x20
}
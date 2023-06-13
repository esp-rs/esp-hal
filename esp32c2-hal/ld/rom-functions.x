PROVIDE(ets_delay_us = 0x40000044);
PROVIDE(ets_update_cpu_frequency_rom = 0x40000774);
PROVIDE(rom_i2c_writeReg = 0x400022f4);
PROVIDE(rom_i2c_writeReg_Mask = 0x400022fc);
PROVIDE(rtc_get_reset_reason = 0x40000018);
PROVIDE(software_reset = 0x40000088);
PROVIDE(software_reset_cpu = 0x4000008c);

PROVIDE(esp_rom_crc32_be = 0x40000808);
PROVIDE(esp_rom_crc16_be = 0x4000080c);
PROVIDE(esp_rom_crc8_be = 0x40000810);
PROVIDE(esp_rom_crc32_le = 0x400007fc);
PROVIDE(esp_rom_crc16_le = 0x40000800);
PROVIDE(esp_rom_crc8_le = 0x40000804);

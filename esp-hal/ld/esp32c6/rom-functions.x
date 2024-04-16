ets_printf = 0x40000028;
PROVIDE(esp_rom_printf = ets_printf);
PROVIDE(cache_invalidate_icache_all = 0x4000064c);
PROVIDE(cache_suspend_icache = 0x40000698);
PROVIDE(cache_resume_icache = 0x4000069c);
PROVIDE(ets_delay_us = 0x40000040);
PROVIDE(ets_update_cpu_frequency_rom = 0x40000048);
PROVIDE(rtc_get_reset_reason = 0x40000018);
ets_update_cpu_frequency = 0x40000048;
PROVIDE(software_reset = 0x40000090);
PROVIDE(software_reset_cpu = 0x40000094);

PROVIDE(esp_rom_crc32_be = 0x40000764);
PROVIDE(esp_rom_crc16_be = 0x40000768);
PROVIDE(esp_rom_crc8_be = 0x4000076c);
PROVIDE(esp_rom_crc32_le = 0x40000758);
PROVIDE(esp_rom_crc16_le = 0x4000075c);
PROVIDE(esp_rom_crc8_le = 0x40000760);

PROVIDE(esp_rom_md5_init = 0x4000074c);
PROVIDE(esp_rom_md5_update = 0x40000750);
PROVIDE(esp_rom_md5_final = 0x40000754);

memset = 0x400004a8;
memcpy = 0x400004ac;
memmove = 0x400004b0;
memcmp = 0x400004b4;

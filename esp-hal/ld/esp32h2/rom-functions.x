ets_printf = 0x40000028;
PROVIDE(esp_rom_printf = ets_printf);
PROVIDE(cache_invalidate_icache_all = 0x40000620);
PROVIDE(cache_suspend_icache = 0x4000066c);
PROVIDE(cache_resume_icache = 0x40000670);
PROVIDE(ets_delay_us = 0x40000040);
PROVIDE(ets_update_cpu_frequency_rom = 0x40000048);
PROVIDE(rtc_get_reset_reason = 0x40000018);
ets_update_cpu_frequency = 0x40000048;
PROVIDE(software_reset = 0x40000090);
PROVIDE(software_reset_cpu = 0x40000094);

PROVIDE(esp_rom_crc32_be = 0x40000730);
PROVIDE(esp_rom_crc16_be = 0x40000734);
PROVIDE(esp_rom_crc8_be = 0x40000738);
PROVIDE(esp_rom_crc32_le = 0x40000724);
PROVIDE(esp_rom_crc16_le = 0x40000728);
PROVIDE(esp_rom_crc8_le = 0x4000072c);

PROVIDE(esp_rom_md5_init = 0x40000718);
PROVIDE(esp_rom_md5_update = 0x4000071c);
PROVIDE(esp_rom_md5_final = 0x40000720);

memset = 0x400004a0;
memcpy = 0x400004a4;
memmove = 0x400004a8;
memcmp = 0x400004ac;

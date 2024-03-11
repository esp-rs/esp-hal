PROVIDE(ets_delay_us = 0x4000d888);
PROVIDE(ets_update_cpu_frequency_rom = 0x4000d8a4);
PROVIDE(rom_i2c_writeReg = 0x4000a9a8);
PROVIDE(rom_i2c_writeReg_Mask = 0x4000aa00);
PROVIDE(rtc_get_reset_reason = 0x4000ff58);
PROVIDE(software_reset = 0x40010068);
PROVIDE(software_reset_cpu = 0x40010080);

PROVIDE ( cache_dbus_mmu_set = 0x40018eb0 );
PROVIDE ( Cache_Allocate_SRAM = 0x40018d6c );
PROVIDE ( Cache_Invalidate_DCache_All = 0x4001842c );
PROVIDE ( Cache_Set_DCache_Mode = 0x40018074 );
PROVIDE ( ets_efuse_get_spiconfig = 0x4000e4a0 );
PROVIDE ( esp_rom_efuse_get_flash_gpio_info    = ets_efuse_get_spiconfig );
PROVIDE ( esp_rom_efuse_get_flash_wp_gpio      = ets_efuse_get_wp_pad );
PROVIDE ( ets_efuse_get_wp_pad = 0x4000e444 );
PROVIDE ( esp_rom_spiflash_select_qio_pins = SelectSpiQIO );
PROVIDE ( SelectSpiQIO = 0x40015b88 );
PROVIDE ( esp_rom_spi_set_op_mode = 0x400179e8 );
PROVIDE ( esp_rom_spi_cmd_start = 0x40017ba8 );
PROVIDE ( esp_rom_spi_cmd_config = 0x40017c58 );

PROVIDE(esp_rom_crc32_le = 0x400119dc);
PROVIDE(esp_rom_crc16_le = 0x40011a10);
PROVIDE(esp_rom_crc8_le = 0x40011a4c);

PROVIDE(esp_rom_md5_final = 0x4000530c);
PROVIDE(esp_rom_md5_init = 0x4000526c);
PROVIDE(esp_rom_md5_update = 0x4000528c);

memcmp = 0x4001ab40;
memcpy = 0x4001aba8;
memmove = 0x4001acb0;
memset = 0x4001ad3c;
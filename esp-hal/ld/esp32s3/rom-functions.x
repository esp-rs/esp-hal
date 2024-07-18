PROVIDE(ets_delay_us = 0x40000600);
PROVIDE(ets_update_cpu_frequency_rom = 0x40001a4c);
PROVIDE(rom_i2c_writeReg = 0x40005d60);
PROVIDE(rom_i2c_writeReg_Mask = 0x40005d6c);
PROVIDE(rtc_get_reset_reason = 0x4000057c);
PROVIDE(rom_config_instruction_cache_mode = 0x40001a1c);
PROVIDE(software_reset = 0x400006d8);
PROVIDE(software_reset_cpu = 0x400006e4);
PROVIDE(cache_dbus_mmu_set = 0x400019b0);
PROVIDE(ets_efuse_get_spiconfig = 0x40001f74);
PROVIDE(esp_rom_efuse_get_flash_gpio_info = ets_efuse_get_spiconfig);
PROVIDE(esp_rom_efuse_get_flash_wp_gpio = ets_efuse_get_wp_pad);
PROVIDE(esp_rom_spiflash_select_qio_pins = 0x40000a68 );
PROVIDE(esp_rom_spi_set_op_mode = 0x400008a0 );
PROVIDE(esp_rom_spi_cmd_start = 0x40000888);
PROVIDE(esp_rom_spi_cmd_config = 0x4000087c);
PROVIDE(Cache_Invalidate_Addr = 0x400016b0 );
PROVIDE(Cache_Suspend_DCache = 0x400018b4 );
PROVIDE(Cache_Resume_DCache = 0x400018c0 );
PROVIDE(Cache_WriteBack_Addr = 0x400016c8 );
PROVIDE(rom_config_data_cache_mode = 0x40001a28 );
PROVIDE(rom_config_instruction_cache_mode = 0x40001a1c );
PROVIDE(ets_efuse_get_wp_pad = 0x40001fa4);
PROVIDE(ets_set_appcpu_boot_addr = 0x40000720);

PROVIDE(esp_rom_crc32_be = 0x40001ca4);
PROVIDE(esp_rom_crc16_be = 0x40001cbc);
PROVIDE(esp_rom_crc8_be = 0x40001cd4);
PROVIDE(esp_rom_crc32_le = 0x40001c98);
PROVIDE(esp_rom_crc16_le = 0x40001cb0);
PROVIDE(esp_rom_crc8_le = 0x40001cc8);

PROVIDE(esp_rom_md5_init = 0x40001c5c);
PROVIDE(esp_rom_md5_update = 0x40001c68);
PROVIDE(esp_rom_md5_final = 0x40001c74);

PROVIDE (esp_rom_opiflash_exec_cmd = 0x400008b8);
PROVIDE( esp_rom_spi_set_dtr_swap_mode = 0x4000093c );
PROVIDE( esp_rom_opiflash_pin_config = 0x40000894 );

memset = 0x400011e8;
memcpy = 0x400011f4;
memmove = 0x40001200;
memcmp = 0x4000120c;

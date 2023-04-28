PROVIDE(ets_delay_us = 0x40008534);
PROVIDE(ets_update_cpu_frequency_rom = 0x40008550);
PROVIDE(rom_i2c_writeReg = 0x400041a4);
PROVIDE(rom_i2c_writeReg_Mask = 0x400041fc);
PROVIDE(rtc_get_reset_reason = 0x400081d4);
PROVIDE(software_reset = 0x4000824c);
PROVIDE(software_reset_cpu = 0x40008264);

PROVIDE ( ets_efuse_get_spiconfig = 0x40008658 );
PROVIDE ( esp_rom_efuse_get_flash_gpio_info    = ets_efuse_get_spiconfig );
PROVIDE ( esp_rom_gpio_connect_out_signal = gpio_matrix_out );
PROVIDE ( gpio_matrix_out = 0x40009f0c );
PROVIDE ( gpio_matrix_in = 0x40009edc );
PROVIDE ( esp_rom_gpio_connect_in_signal  = gpio_matrix_in );
PROVIDE ( esp_rom_spiflash_config_clk = 0x40062bc8 );
PROVIDE ( g_rom_spiflash_dummy_len_plus = 0x3ffae290 );
PROVIDE ( g_rom_flashchip = 0x3ffae270 );
PROVIDE ( cache_sram_mmu_set_rom = 0x400097f4 );

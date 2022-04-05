ets_printf = 0x40000040;
PROVIDE(esp_rom_printf = ets_printf);
PROVIDE(cache_invalidate_icache_all = 0x400004d8);
PROVIDE(cache_suspend_icache = 0x40000524);
PROVIDE(cache_resume_icache = 0x40000528);
PROVIDE(cache_ibus_mmu_set = 0x40000560);
PROVIDE(cache_dbus_mmu_set = 0x40000564);

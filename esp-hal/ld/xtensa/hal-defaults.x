PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);
PROVIDE(__init_persistent = default_mem_hook);
PROVIDE(__post_init = no_init_hook);

/* Called at the end of `__post_init`, right before `main`. A bootloader support crate may define it. */
PROVIDE(__esp_bootloader_init = __esp_bootloader_default_init);

/* Called before a deep sleep. esp-phy may define it to power down the RF. */
PROVIDE(__esp_phy_deep_sleep_prepare = __esp_phy_default_deep_sleep_prepare);

PROVIDE(DefaultHandler = EspDefaultHandler);

PROVIDE(level1_interrupt = DefaultHandler);
PROVIDE(level2_interrupt = DefaultHandler);
PROVIDE(level3_interrupt = DefaultHandler);
PROVIDE(level4_interrupt = DefaultHandler);
PROVIDE(level5_interrupt = DefaultHandler);
PROVIDE(level6_interrupt = DefaultHandler);
PROVIDE(level7_interrupt = DefaultHandler);

/* external interrupts (from PAC) */
INCLUDE "device.x"

INCLUDE "esp_hal_checks.x"

INCLUDE "esp_hal_config.x"
INCLUDE "direct_boot_memory.x"

/* esp-hal region aliases */
REGION_ALIAS("ROTEXT", ROM);
REGION_ALIAS("RODATA", ROM);

REGION_ALIAS("RWTEXT", RAM);
REGION_ALIAS("RWDATA", RAM);

/* riscv-rt v0.16+ region aliases */
REGION_ALIAS("REGION_TEXT", ROM);
REGION_ALIAS("REGION_RODATA", ROM);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

PROVIDE(_max_hart_id = 1);

/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, this chip also has ID 0. */
PROVIDE(interrupt0 = DefaultHandler);

/* Called from all harts, returns true only for the one that initializes memory. */
PROVIDE(_mp_hook = default_mp_hook);

INCLUDE "direct_boot_header.x"
_direct_boot_rwtext_lma = _direct_boot_ram_lma;
INCLUDE "direct_boot_ram.x"
INCLUDE "direct_boot_rtc_fast.x"
INCLUDE "direct_boot_flash.x"
INCLUDE "direct_boot_common.x"
INCLUDE "hal-defaults.x"

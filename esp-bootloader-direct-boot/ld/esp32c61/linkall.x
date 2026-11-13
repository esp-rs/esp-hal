INCLUDE "esp_hal_config.x"
INCLUDE "direct_boot_memory.x"

REGION_ALIAS("ROTEXT", ROM);
REGION_ALIAS("RODATA", ROM);

REGION_ALIAS("RWTEXT", RAM);
REGION_ALIAS("RWDATA", RAM);

/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, this chip also has ID 0. */
PROVIDE(interrupt0 = DefaultHandler);

INCLUDE "direct_boot_header.x"
_direct_boot_rwtext_lma = _direct_boot_ram_lma;
INCLUDE "direct_boot_ram.x"
_direct_boot_flash_lma = _direct_boot_data_end_lma;
INCLUDE "direct_boot_flash.x"
INCLUDE "direct_boot_common.x"
INCLUDE "hal-defaults.x"

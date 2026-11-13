INCLUDE "esp_hal_config.x"
INCLUDE "direct_boot_memory.x"

REGION_ALIAS("ROTEXT", IROM);
REGION_ALIAS("RODATA", DROM);

REGION_ALIAS("RWTEXT", IRAM);
REGION_ALIAS("RWDATA", DRAM);

INCLUDE "direct_boot_header.x"
_direct_boot_rwtext_lma = _direct_boot_ram_lma;
INCLUDE "direct_boot_ram.x"
INCLUDE "direct_boot_rtc_fast.x"
INCLUDE "direct_boot_flash.x"
INCLUDE "direct_boot_common.x"
INCLUDE "hal-defaults.x"

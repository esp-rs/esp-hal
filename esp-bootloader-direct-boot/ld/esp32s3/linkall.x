INCLUDE "esp_hal_config.x"
INCLUDE "direct_boot_memory.x"

REGION_ALIAS("ROTEXT", irom_seg);
REGION_ALIAS("RODATA", drom_seg);
REGION_ALIAS("RWTEXT", iram_seg);
REGION_ALIAS("RWDATA", dram_seg);

INCLUDE "direct_boot_layout.x"
INCLUDE "hal-defaults.x"

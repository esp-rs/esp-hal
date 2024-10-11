INCLUDE "memory.x"

REGION_ALIAS("ROTEXT", IROM);
REGION_ALIAS("RODATA", DROM);

REGION_ALIAS("RWDATA", DRAM);
REGION_ALIAS("RWTEXT", IRAM);

INCLUDE "esp32c2.x"
INCLUDE "hal-defaults.x"
INCLUDE "rom-functions.x"

#IF __include_rom_functions
    INCLUDE "rom_functions.x"
#ENDIF

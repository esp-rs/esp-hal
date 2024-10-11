INCLUDE "memory.x"

REGION_ALIAS("ROTEXT", ROM);
REGION_ALIAS("RODATA", ROM);

REGION_ALIAS("RWTEXT", RAM);
REGION_ALIAS("RWDATA", RAM);

REGION_ALIAS("RTC_FAST_RWTEXT", RTC_FAST);
REGION_ALIAS("RTC_FAST_RWDATA", RTC_FAST);

INCLUDE "esp32h2.x"
INCLUDE "hal-defaults.x"
INCLUDE "rom-functions.x"

#IF __include_rom_functions
    INCLUDE "rom_functions.x"
#ENDIF

#IF __include_rom_coexist_and_phy
    INCLUDE "rom_coexist.x"
    INCLUDE "rom_phy.x"
#ENDIF

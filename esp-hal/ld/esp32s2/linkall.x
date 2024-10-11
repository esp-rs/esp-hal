INCLUDE "memory.x"
INCLUDE "alias.x"
INCLUDE "esp32s2.x"
INCLUDE "hal-defaults.x"
INCLUDE "rom-functions.x"

#IF __include_rom_functions
    INCLUDE "rom_functions.x"
#ENDIF

#IF __include_rom_coexist_and_phy
    INCLUDE "rom_coexist.x"
    INCLUDE "rom_phy.x"
#ENDIF

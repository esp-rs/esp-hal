#IF ESP_HAL_CONFIG_PLACE_SWITCH_TABLES_IN_RAM
*(.rodata.*_esp_hal_internal_handler*)
*(.rodata..Lswitch.table.*)
*(.rodata.cst*)
#ENDIF

#IF ESP_HAL_CONFIG_PLACE_ANON_IN_RAM
*(.rodata..Lanon .rodata..Lanon.*)
#ENDIF

#IF psram_separate_window && ESP_HAL_CONFIG_CODE_IN_PSRAM
/* The start-up reads these before `.rodata` is in PSRAM, so they stay in RAM whatever
   `place-switch-tables-in-ram` is set to. */
*(.rodata..Lswitch.table.* .rodata.cst*)
#ENDIF

#IF ESP_HAL_CONFIG_USE_RWDATA_LD_HOOK
INCLUDE "rwdata_hook.x"
#ENDIF

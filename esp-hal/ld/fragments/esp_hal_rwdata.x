#IF ESP_HAL_CONFIG_PLACE_SWITCH_TABLES_IN_RAM
*(.rodata.*_esp_hal_internal_handler*)
*(.rodata..Lswitch.table.*)
*(.rodata.cst*)
#ENDIF

#IF ESP_HAL_CONFIG_PLACE_ANON_IN_RAM
*(.rodata..Lanon .rodata..Lanon.*)
#ENDIF

#IF ESP_HAL_CONFIG_USE_RWDATA_LD_HOOK
INCLUDE "rwdata_hook.x"
#ENDIF

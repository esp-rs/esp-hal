SECTIONS {
  .rwtext : ALIGN(4)
  {
    . = ALIGN (4);
    #IF ESP_HAL_PLACE_MORE_RODATA_IN_RAM
      *(.rodata.*_esp_hal_internal_handler*)
      *(.rodata.str.*)
      *(.rodata.handle_interrupts)
      *(.rodata..Lswitch.table.*)
    #ENDIF    
    *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)
    . = ALIGN(4);
  } > RWTEXT

  .rwtext.wifi :
  {
    . = ALIGN(4);
    *( .wifi0iram  .wifi0iram.*)
    *( .wifirxiram  .wifirxiram.*)
    *( .wifislprxiram  .wifislprxiram.*)
    *( .wifislpiram  .wifislpiram.*)
    *( .phyiram  .phyiram.*)
    *( .iram1  .iram1.*)
    *( .wifiextrairam.* )
    *( .coexiram.* )
    . = ALIGN(4);
  } > RWTEXT AT > RODATA
}
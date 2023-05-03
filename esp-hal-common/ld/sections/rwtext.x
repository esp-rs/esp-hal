

SECTIONS {
  .rwtext : ALIGN(4)
  {
    . = ALIGN (4);
    *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)
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
  } > RWTEXT AT > RODATA
}
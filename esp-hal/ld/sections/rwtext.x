.rwtext : ALIGN(4)
{
  . = ALIGN (4);
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

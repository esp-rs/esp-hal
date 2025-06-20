#IF riscv
.trap : ALIGN(4)
{
  KEEP(*(.trap));
  *(.trap.*);
} > RWTEXT
#ENDIF

.rwtext : ALIGN(4)
{
  . = ALIGN (4);
  *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)
  ${RWTEXT_ADDITION}
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
} > RWTEXT

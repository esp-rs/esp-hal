.data : ALIGN(4)
{
  _data_start = ABSOLUTE(.);
  . = ALIGN (4);

  INCLUDE "esp_hal_rwdata.x"

  *(.sdata .sdata.* .sdata2 .sdata2.*);
  *(.data .data.*);
  *(.data1)
  _data_end = ABSOLUTE(.);
  . = ALIGN(4);
} > RWDATA

/* LMA of .data */
_sidata = LOADADDR(.data);

.data.wifi :
{
  . = ALIGN(4);
  *( .dram1 .dram1.*)
  . = ALIGN(4);
} > RWDATA

.bss (NOLOAD) : ALIGN(4)
{
  _bss_start = ABSOLUTE(.);
  . = ALIGN (4);
  *(.dynsbss)
  *(.sbss)
  *(.sbss.*)
  *(.gnu.linkonce.sb.*)
  *(.scommon)
  *(.sbss2)
  *(.sbss2.*)
  *(.gnu.linkonce.sb2.*)
  *(.dynbss)
  *(.sbss .sbss.* .bss .bss.*);
  *(.share.mem)
  *(.gnu.linkonce.b.*)
  *(COMMON)
  _bss_end = ABSOLUTE(.);
  . = ALIGN(4);
} > RWDATA

.noinit (NOLOAD) : ALIGN(4)
{
  . = ALIGN(4);
  *(.noinit .noinit.*)
  *(.uninit .uninit.*)
  . = ALIGN(4);

  _noinit_end = .;
} > RWDATA



SECTIONS {
  .rodata : ALIGN(4)
  {
    _rodata_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.rodata .rodata.*)
    _rodata_end = ABSOLUTE(.);
  } > RODATA

  .rodata.wifi :
  {
    . = ALIGN(4);
    *( .rodata_wlog_*.* )
  } > RODATA
}
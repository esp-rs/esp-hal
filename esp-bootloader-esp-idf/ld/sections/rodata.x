SECTIONS {
  /* For ESP App Description, must be placed first in image */
  .flash.appdesc : ALIGN(4)
  {
      KEEP(*(.flash.appdesc));
      KEEP(*(.flash.appdesc.*));
  } > RODATA

  /*
    Depending on the input sections which go into .rodata the alignment might get adjusted to >4.
    Make sure we have a "merge-section" here to make the tooling NOT emit more than two sections which go into flash.
  */
  .rodata_merge : ALIGN (4) {
    . = ALIGN(ALIGNOF(.rodata));
  } > RODATA

  .rodata : ALIGN(4)
  {
    . = ALIGN (4);
    _rodata_start = ABSOLUTE(.);
    *(.rodata .rodata.*)
    *(.srodata .srodata.*)
    . = ALIGN(4);
    _rodata_end = ABSOLUTE(.);
  } > RODATA

  .rodata.wifi : ALIGN(4)
  {
    . = ALIGN(4);
    *( .rodata_wlog_*.* )
    . = ALIGN(4);
  } > RODATA
}

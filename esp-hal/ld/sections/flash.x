SECTIONS {
  /* For ESP App Description, must be placed first in image */
  .flash.appdesc : ALIGN(4)
  {
      KEEP(*(.flash.appdesc));
      KEEP(*(.flash.appdesc.*));
  } > RODATA
}

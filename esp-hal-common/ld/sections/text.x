

SECTIONS {

  .text : ALIGN(4)
  {
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.text.abort));
    *(.literal .text .literal.* .text.*)
  } > ROTEXT

}
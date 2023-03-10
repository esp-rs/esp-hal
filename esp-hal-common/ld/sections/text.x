

SECTIONS {

  .text : ALIGN(4)
  {
    *(.literal .text .literal.* .text.*)
  } > ROTEXT

}
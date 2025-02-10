

SECTIONS {

  .text : ALIGN(4)
  {
    KEEP(*(.rodata_desc));

    #IF riscv
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.text.abort));
    #ENDIF
    *(.literal .text .literal.* .text.*)
  } > ROTEXT

}
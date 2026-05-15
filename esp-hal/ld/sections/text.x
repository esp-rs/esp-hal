

SECTIONS {

  .text : ALIGN(4)
  {
    #IF riscv
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.text.abort));
    #ENDIF
    /* Xtensa L32R loads literals at PC-relative *negative* offsets, so all
       literal pools must appear before the code that references them. GNU ld
       reorders sections within a single `*(...)` clause by pattern position;
       LLD preserves input order, so we split the patterns to force the right
       layout under both linkers. */
    *(.literal .literal.*)
    *(.text .text.*)
  } > ROTEXT

}


SECTIONS {

  .text : ALIGN(4)
  {
    INCLUDE "text_init.x"

    INCLUDE "esp_hal_text_radio.x"

    /* Bare `.literal` (no `.<f>` suffix) holds the `global_asm!` literal
       pools used by the boot code (`__pre_init` / `__post_init`). Unlike
       Rust functions, those `.text.*` sections do not carry their own
       paired `.literal.<f>`, so place the bare pool first and pull the two
       consumers in right behind it — that keeps their L32R loads in reach
       regardless of total image size. */
    *(.literal)
    *(.text.__post_init)
    *(.text.__pre_init)

    /* Catch-all (mostly Rust code). rustc's Xtensa backend emits each
       function's `.literal.<f>` immediately before its `.text.<f>` within
       the object, so — like the per-archive blocks above — a single
       combined `*(...)` group keeps every literal pool within L32R reach
       of its code. Splitting literals and text into separate `*(...)`
       groups instead bunches all literals at the start and pushes late
       code past the 256 KB L32R window in large (radio) images. */
    *(.literal.* .text .text.*)
  } > ROTEXT

}
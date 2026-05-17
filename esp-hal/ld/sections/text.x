

SECTIONS {

  .text : ALIGN(4)
  {
    #IF riscv
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.text.abort));
    #ENDIF

    /* The Espressif blobs (esp-wifi-sys / esp-phy / BT) are GCC-compiled and
       emit per-function `.literal.<f>` paired with `.text.<f>` within each
       object file. Pull each archive's sections in as a single block using
       the combined pattern: both GNU ld and LLD preserve input-file order
       within a single `*(...)` group, which keeps every function's literal
       pool within L32R reach of its own code regardless of total image size.
       Without these per-archive selectors, the catch-all below would bunch
       all literals at the start and push wifi text past the 256 KB L32R
       window. */
    *libpp.a:(.literal .text .literal.* .text.*)
    *libnet80211.a:(.literal .text .literal.* .text.*)
    *libwpa_supplicant.a:(.literal .text .literal.* .text.*)
    *libmesh.a:(.literal .text .literal.* .text.*)
    *libsmartconfig.a:(.literal .text .literal.* .text.*)
    *libphy.a:(.literal .text .literal.* .text.*)
    *libcoexist.a:(.literal .text .literal.* .text.*)
    *libcore.a:(.literal .text .literal.* .text.*)
    *libregulatory.a:(.literal .text .literal.* .text.*)
    *libespnow.a:(.literal .text .literal.* .text.*)
    *libwapi.a:(.literal .text .literal.* .text.*)
    *libprintf.a:(.literal .text .literal.* .text.*)
    *libbtdm_app.a:(.literal .text .literal.* .text.*)
    *libbtbb.a:(.literal .text .literal.* .text.*)

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
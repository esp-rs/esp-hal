KEEP(*(.init));
KEEP(*(.init.rust));
KEEP(*(.hal_main));
/* build.rs writes this file. It aligns `_pre_init_trap` on a chip with a CLIC. */
INCLUDE "pre_init_trap_align.x"
KEEP(*(.text.abort));

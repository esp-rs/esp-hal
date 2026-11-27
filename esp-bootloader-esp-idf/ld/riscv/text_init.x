KEEP(*(.init));
KEEP(*(.init.rust));
#IF soc_has_clic
/* The start-up first sets `mtvec` to `_pre_init_trap`. With a CLIC, the trap base ignores the low
   6 bits of `mtvec`. */
. = ALIGN(64);
#ENDIF
KEEP(*(.text.abort));

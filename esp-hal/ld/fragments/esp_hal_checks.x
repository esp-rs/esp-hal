/* Flash operations and light sleep run `.rwtext.cache_suspend_safe` with the caches suspended. In
   that time, code and constants in flash are not readable. If the section calls a helper in
   flash, the link fails here and names the helper. Code in `.rwtext` can call flash, so the
   section must not call `.rwtext` either. LLD checks every reference. GNU ld checks only
   references to global symbols, so most calls to Rust functions and reads of constants pass. */
NOCROSSREFS_TO(.text .rwtext.cache_suspend_safe)
NOCROSSREFS_TO(.rodata .rwtext.cache_suspend_safe)
NOCROSSREFS_TO(.rwtext .rwtext.cache_suspend_safe)

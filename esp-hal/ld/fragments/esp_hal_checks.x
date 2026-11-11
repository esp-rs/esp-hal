/* Code in `.rwtext.checked` runs while flash cannot be read, for example while flash operations
   and light sleep suspend the caches. If the section refers to code or constants in flash, the
   link fails here and names the symbol. Code in `.rwtext` can call flash, so the section must not
   call `.rwtext` either. The link finds direct references only, not a call through a function
   pointer. LLD finds every direct reference. GNU ld finds only references to global symbols, so
   most calls to Rust functions and reads of constants pass. */
NOCROSSREFS_TO(.text .rwtext.checked)
NOCROSSREFS_TO(.rodata .rwtext.checked)
NOCROSSREFS_TO(.rwtext .rwtext.checked)

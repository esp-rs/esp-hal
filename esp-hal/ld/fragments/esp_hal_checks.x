/* Code in `.rwtext.checked` runs while flash cannot be read, for example while flash operations
   and light sleep suspend the caches. If the section refers to code or constants in flash, the
   link fails here and names the symbol. Code in `.rwtext` can call flash, so the section must not
   call `.rwtext` either. The link finds direct references only, not a call through a function
   pointer. LLD finds every direct reference. GNU ld finds only references to global symbols, so
   most calls to Rust functions and reads of constants pass. */
NOCROSSREFS_TO(.text .rwtext.checked)
NOCROSSREFS_TO(.rodata .rwtext.checked)
NOCROSSREFS_TO(.rwtext .rwtext.checked)

#IF ESP_HAL_CONFIG_CODE_IN_PSRAM
/* `.init` runs before `.text` and `.rodata` are in PSRAM. Code in `.rwtext` can call `.text`, so
   `.init` must not call `.rwtext` either. */
NOCROSSREFS_TO(.text .init)
NOCROSSREFS_TO(.rodata .init)
NOCROSSREFS_TO(.rwtext .init)

/* The entry code must be in `.init`, which stays in flash. `_abs_start` is a local label after
   `_start` in the same input section, so no `ASSERT` names it. */
_esp_hal_init_end = ADDR(.init) + SIZEOF(.init);
ASSERT(_start >= ADDR(.init) && _start < _esp_hal_init_end, "With code-in-psram, _start must be in .init");
ASSERT(__pre_init >= ADDR(.init) && __pre_init < _esp_hal_init_end, "With code-in-psram, __pre_init must be in .init");
#ENDIF

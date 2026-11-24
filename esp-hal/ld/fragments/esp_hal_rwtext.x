#IF rom_no_libgcc
/* Without libgcc in ROM, compiler intrinsics must stay callable while the flash cache is disabled. */
*libcompiler_builtins*:*(.literal .text .literal.* .text.*)
#ENDIF

#IF psram_separate_window && ESP_HAL_CONFIG_CODE_IN_PSRAM
/* The start-up from `_start` runs before the code is in PSRAM. */
KEEP(*(.init));
KEEP(*(.init.rust));
KEEP(*(.text.abort));
#ENDIF

#IF ESP_HAL_CONFIG_USE_RWTEXT_LD_HOOK
INCLUDE "rwtext_hook.x"
#ENDIF

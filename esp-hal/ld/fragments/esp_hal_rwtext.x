#IF rom_no_libgcc
/* Without libgcc in ROM, compiler intrinsics must stay callable while the flash cache is disabled. */
*libcompiler_builtins*:*(.literal .text .literal.* .text.*)
#ENDIF

#IF ESP_HAL_CONFIG_USE_RWTEXT_LD_HOOK
INCLUDE "rwtext_hook.x"
#ENDIF

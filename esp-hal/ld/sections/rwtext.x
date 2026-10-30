#IF riscv
.trap : ALIGN(4)
{
  _trap_section_origin = .;
  KEEP(*(.trap));
  *(.trap.*);

  *(.rwtext.interrupt .rwtext.interrupt.*)
} > RWTEXT
#ENDIF

/* Code that runs while the caches are suspended. `text.x` forbids references from this section
   to flash and to `.rwtext`. This section must come before `.rwtext`, or `.rwtext` takes the
   input sections. */
.rwtext.cache_suspend_safe : ALIGN(4)
{
  _rwtext_start = .;
  *(.rwtext.cache_suspend_safe.*)
  /* unconditionally add patched SPI-flash ROM functions (from esp-rom-sys) - the linker is still happy if there are none */
  *:esp_rom_spiflash.*(.literal .text .literal.* .text.*)
  . = ALIGN(4);
} > RWTEXT

.rwtext : ALIGN(4)
{
  . = ALIGN (4);

  /* Keep each object's literal pool adjacent to its code (see comment in `text.x`). */
  *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)

  #IF rom_no_libgcc
    /* Without libgcc in ROM, compiler intrinsics must stay callable while the flash cache is disabled. */
    *libcompiler_builtins*:*(.literal .text .literal.* .text.*)
  #ENDIF

  #IF ESP_HAL_CONFIG_USE_RWTEXT_LD_HOOK
    INCLUDE "rwtext_hook.x"
  #ENDIF

  . = ALIGN(4);
} > RWTEXT

.rwtext.wifi :
{
  . = ALIGN(4);
  *( .wifi0iram  .wifi0iram.*)
  *( .wifirxiram  .wifirxiram.*)
  *( .wifislprxiram  .wifislprxiram.*)
  *( .wifislpiram  .wifislpiram.*)
  *( .phyiram  .phyiram.*)
  *( .iram1  .iram1.*)
  *( .wifiextrairam.* )
  *( .coexiram.* )
  *( .high_perf_code_iram* )
  *( .coexsleepiram* )
  *( .wifiorslpiram* )
  *( .isr_iram* )
  *( .conn_iram* )
  *( .sleep_iram* )
  . = ALIGN(4);

  _rwtext_end = .;
  _rwtext_len = . - ORIGIN(RWTEXT);
} > RWTEXT

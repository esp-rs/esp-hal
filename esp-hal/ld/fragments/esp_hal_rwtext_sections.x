/* Output sections in RWTEXT only. The bootloader support crate includes this file inside `SECTIONS`,
   in front of the output section that holds `*(.rwtext.*)`, so these sections get their input
   sections first. */

/* Code that runs while flash cannot be read. `esp_hal_checks.x` forbids references from this
   section to flash and to `.rwtext`. */
.rwtext.checked : ALIGN(4)
{
  *(.rwtext.checked.*)
  /* unconditionally add patched SPI-flash ROM functions (from esp-rom-sys) - the linker is still happy if there are none */
  *:esp_rom_spiflash.*(.literal .text .literal.* .text.*)
  . = ALIGN(4);
} > RWTEXT

#IF ESP_HAL_CONFIG_CODE_IN_PSRAM
/* The start-up up to the copy into PSRAM, except `hal_main`. It runs from flash. See
   `esp_hal_checks.x`. */
.init : ALIGN(4)
{
  KEEP(*(.init));
  KEEP(*(.init.rust));
  . = ALIGN(4);
} > BOOT_FLASH

/* `hal_main` calls `.text` after the copy, so the check does not cover it. */
.hal_main : ALIGN(4)
{
  KEEP(*(.hal_main));
  . = ALIGN(4);
} > BOOT_FLASH

/* `_pre_init_trap`, in RAM, because flash is unmapped before the interrupts are set up. A CLIC
   `mtvec` needs a base aligned to 64 bytes. */
.abort : ALIGN(64)
{
  KEEP(*(.text.abort));
  . = ALIGN(4);
} > RWTEXT
#ENDIF

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

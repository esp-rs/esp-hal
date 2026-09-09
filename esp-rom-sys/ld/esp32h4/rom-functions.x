/* Taken from esp-idf commit 2067f3ae32b.
   The ESP32-H4 ROM does not provide heap (TLSF) or RVFP linker scripts. */
INCLUDE "rom/esp32h4.rom.api.ld"
INCLUDE "rom/esp32h4.rom.ld"
INCLUDE "rom/esp32h4.rom.libgcc.ld"
INCLUDE "rom/esp32h4.rom.spiflash.ld"
INCLUDE "rom/esp32h4.rom.version.ld"

INCLUDE "rom/additional.ld"

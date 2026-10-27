/* Taken from esp-idf commit 4d80c1b29ae.
   The ESP32-H4 ROM does not provide heap (TLSF) or RVFP linker scripts.
   Since ECO2 (v0.2), the ROM does not provide libgcc functions either. */
INCLUDE "rom/esp32h4.rom.api.ld"
INCLUDE "rom/esp32h4.rom.ld"
INCLUDE "rom/esp32h4.rom.spiflash.ld"
INCLUDE "rom/esp32h4.rom.version.ld"

INCLUDE "rom/additional.ld"

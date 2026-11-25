/* Taken from esp-idf commit 4d80c1b29ae.
   The ESP32-H21 ROM does not provide a heap (TLSF) linker script. */
INCLUDE "rom/esp32h21.rom.api.ld"
INCLUDE "rom/esp32h21.rom.ld"
INCLUDE "rom/esp32h21.rom.libgcc.ld"
INCLUDE "rom/esp32h21.rom.spiflash.ld"
INCLUDE "rom/esp32h21.rom.version.ld"

INCLUDE "rom/additional.ld"

INCLUDE "rom/esp32s3.rom.api.ld"
INCLUDE "rom/esp32s3.rom.ld"
INCLUDE "rom/esp32s3.rom.libgcc.ld"
INCLUDE "rom/esp32s3.rom.version.ld"

/*
TODO if BT_CTRL_RUN_IN_FLASH_ONLY

INCLUDE "rom/esp32s3.rom.ble_50.ld"
INCLUDE "rom/esp32s3.rom.ble_cca.ld"
INCLUDE "rom/esp32s3.rom.ble_dtm.ld"
INCLUDE "rom/esp32s3.rom.ble_master.ld"
INCLUDE "rom/esp32s3.rom.ble_scan.ld"
INCLUDE "rom/esp32s3.rom.ble_smp.ld"
INCLUDE "rom/esp32s3.rom.ble_test.ld"
*/

INCLUDE "rom/esp32s3.rom.bt_funcs.ld"

INCLUDE "rom/additional.ld"

//! eFuse fields for the ESP32-S31.
//!
//! This file was generated from espefuse/efuse_defs/esp32s31.yaml
//! (via espflash/espflash/src/target/efuse/esp32s31.rs).
//!
//! For information on how to regenerate these files, please refer to the
//! `xtask` package's `README.md` file.
use crate::efuse::EfuseField;

/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// Disable reading from BLOCK4-9
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// Disable USB JTAG function
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Disable USB-Serial-JTAG
pub const DIS_USB_SERIAL_JTAG: EfuseField = EfuseField::new(0, 1, 43, 1);
/// Disable force download mode
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Disable SPI0 during boot_mode_download
pub const SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Disable TWAI
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Enable JTAG selection via strapping pin
pub const JTAG_SEL_ENABLE: EfuseField = EfuseField::new(0, 1, 47, 1);
/// Soft-disable JTAG
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 48, 3);
/// Permanently disable JTAG
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Disable manual flash encryption
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 52, 1);
/// Disable Wi-Fi 6
pub const DIS_WIFI6: EfuseField = EfuseField::new(0, 1, 54, 1);
/// HUK generation state
pub const HUK_GEN_STATE: EfuseField = EfuseField::new(0, 1, 55, 5);
/// KM random number switch cycle control
pub const KM_RND_SWITCH_CYCLE: EfuseField = EfuseField::new(0, 2, 64, 1);
/// Key purpose for Key0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 2, 65, 5);
/// Key purpose for Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 2, 70, 5);
/// Key purpose for Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 2, 75, 5);
/// Key purpose for Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 2, 80, 5);
/// Key purpose for Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 2, 85, 5);
/// Enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 96, 1);
/// Enable aggressive revocation for secure boot
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 97, 1);
/// Flash power-up waiting time
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 100, 4);
/// Disable download mode
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 3, 104, 1);
/// Disable direct boot mode
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 3, 105, 1);
/// Disable USB print from ROM
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 3, 106, 1);
/// Disable USB download mode
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 3, 107, 1);
/// Enable security download
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 3, 108, 1);
/// UART print control
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 3, 109, 2);
/// Enables flash encryption counter
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 3, 111, 3);
/// Secure version for anti-rollback
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 3, 114, 16);
/// Revoke secure boot key 0
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Revoke secure boot key 1
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 4, 129, 1);
/// Revoke secure boot key 2
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 4, 130, 1);
/// `[MAC_FACTORY]` MAC address (bits 0–31)
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// `[MAC_FACTORY]` MAC address (bits 32–47)
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// Minor chip version
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(1, 3, 114, 4);
/// Major chip version
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 118, 2);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 120, 1);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 121, 1);
/// BLK_VERSION_MINOR of BLOCK2
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(1, 3, 122, 3);
/// BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 125, 2);
/// PSRAM capacity
pub const PSRAM_CAP: EfuseField = EfuseField::new(1, 3, 127, 3);
/// Max ambient temperature
pub const TEMP: EfuseField = EfuseField::new(1, 4, 130, 2);
/// PSRAM vendor
pub const PSRAM_VENDOR: EfuseField = EfuseField::new(1, 4, 132, 2);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 4, 134, 2);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// `[BLOCK_USR_DATA]` User data
pub const BLOCK_USR_DATA: EfuseField = EfuseField::new(3, 0, 0, 256);
/// `[MAC_CUSTOM CUSTOM_MAC]` Custom MAC
pub const CUSTOM_MAC: EfuseField = EfuseField::new(3, 6, 200, 48);
/// `[BLOCK_KEY0]` Key0 or user data
pub const BLOCK_KEY0: EfuseField = EfuseField::new(4, 0, 0, 256);
/// `[BLOCK_KEY1]` Key1 or user data
pub const BLOCK_KEY1: EfuseField = EfuseField::new(5, 0, 0, 256);
/// `[BLOCK_KEY2]` Key2 or user data
pub const BLOCK_KEY2: EfuseField = EfuseField::new(6, 0, 0, 256);
/// `[BLOCK_KEY3]` Key3 or user data
pub const BLOCK_KEY3: EfuseField = EfuseField::new(7, 0, 0, 256);
/// `[BLOCK_KEY4]` Key4 or user data
pub const BLOCK_KEY4: EfuseField = EfuseField::new(8, 0, 0, 256);
/// `[BLOCK_SYS_DATA2]` System data part 2 (reserved)
pub const BLOCK_SYS_DATA2: EfuseField = EfuseField::new(9, 0, 0, 256);

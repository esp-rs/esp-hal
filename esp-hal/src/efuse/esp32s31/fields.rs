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
/// Reserved
pub const RESERVE_0_39: EfuseField = EfuseField::new(0, 1, 39, 3);
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
/// Soft-disable JTAG (odd = disabled, even = enabled)
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 48, 3);
/// Permanently disable JTAG
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Disable manual flash encryption
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 52, 1);
/// Reserved
pub const RESERVE_0_53: EfuseField = EfuseField::new(0, 1, 53, 1);
/// Disable Wi-Fi 6
pub const DIS_WIFI6: EfuseField = EfuseField::new(0, 1, 54, 1);
/// HUK generation state (odd count = invalid)
pub const HUK_GEN_STATE: EfuseField = EfuseField::new(0, 1, 55, 5);
/// Reserved
pub const RESERVE_0_60: EfuseField = EfuseField::new(0, 1, 60, 4);
/// KM random number switch cycle control
pub const KM_RND_SWITCH_CYCLE: EfuseField = EfuseField::new(0, 2, 64, 1);
/// Reserved
pub const RESERVE_0_65: EfuseField = EfuseField::new(0, 2, 65, 1);
/// Disable KM deploy modes (per-bit: ecdsa, flash/spi, hmac/aes, ds/rma, psram)
pub const KM_DISABLE_DEPLOY_MODE: EfuseField = EfuseField::new(0, 2, 66, 5);
/// Restrict each KM key to single-deploy (per-bit)
pub const KM_DEPLOY_ONLY_ONCE: EfuseField = EfuseField::new(0, 2, 71, 5);
/// Force use of key manager key (per-bit)
pub const FORCE_USE_KEY_MANAGER_KEY: EfuseField = EfuseField::new(0, 2, 76, 5);
/// Disable software-written init key; force efuse_init_key
pub const FORCE_DISABLE_SW_INIT_KEY: EfuseField = EfuseField::new(0, 2, 81, 1);
/// Configure flash encryption to use XTS-128 key (0 = 128-bit, 1 = 256-bit)
pub const KM_XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(0, 2, 82, 1);
/// RTC watchdog STG0 timeout multiplier
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 2, 83, 1);
/// Disable all SM crypto functions (SM2, SM3)
pub const DIS_SM_CRYPT: EfuseField = EfuseField::new(0, 2, 84, 1);
/// Enables flash encryption counter
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 2, 85, 3);
/// Revoke secure boot key 0
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 2, 88, 1);
/// Revoke secure boot key 1
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 2, 89, 1);
/// Revoke secure boot key 2
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 2, 90, 1);
/// Reserved
pub const RESERVE_0_91: EfuseField = EfuseField::new(0, 2, 91, 5);
/// Key purpose for Key0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 3, 96, 5);
/// Key purpose for Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 3, 101, 5);
/// Key purpose for Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 3, 106, 5);
/// Key purpose for Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 3, 111, 5);
/// Key purpose for Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 3, 116, 5);
/// Permanently enable ECC constant-time mode
pub const ECC_FORCE_CONST_TIME: EfuseField = EfuseField::new(0, 3, 121, 1);
/// Permanently disable ECDSA software-set KEY
pub const ECDSA_DISABLE_SOFT_K: EfuseField = EfuseField::new(0, 3, 122, 1);
/// SPA secure level (clock random divide mode)
pub const SEC_DPA_LEVEL: EfuseField = EfuseField::new(0, 3, 123, 2);
/// Enable XTS clock anti-DPA attack
pub const XTS_DPA_CLK_ENABLE: EfuseField = EfuseField::new(0, 3, 125, 1);
/// Reserved
pub const RESERVE_0_126: EfuseField = EfuseField::new(0, 3, 126, 2);
/// XTS pseudo-round anti-DPA level (0 = register-controlled)
pub const XTS_DPA_PSEUDO_LEVEL: EfuseField = EfuseField::new(0, 4, 128, 2);
/// Enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 4, 130, 1);
/// Enable aggressive revocation for secure boot
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Reserved
pub const RESERVE_0_132: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Flash type (0 = NOR, 1 = NAND)
pub const FLASH_TYPE: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Reserved
pub const RESERVE_0_134: EfuseField = EfuseField::new(0, 4, 134, 3);
/// Disable USB-OTG download mode
pub const DIS_USB_OTG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 137, 1);
/// Reserved
pub const RESERVE_0_138: EfuseField = EfuseField::new(0, 4, 138, 2);
/// Flash power-up waiting time (ms; ≥15 = 2× programmed value)
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 4, 140, 4);
/// Disable download mode
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 144, 1);
/// Disable direct boot mode
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 4, 145, 1);
/// Disable USB print from ROM
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 4, 146, 1);
/// Lock KM efuse key
pub const LOCK_KM_KEY: EfuseField = EfuseField::new(0, 4, 147, 1);
/// Disable USB download mode
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 148, 1);
/// Enable security download
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 149, 1);
/// UART print control (00=force on, 01=GPIO8 low, 10=GPIO8 high, 11=force off)
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 150, 2);
/// Force ROM to send resume command during SPI boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 152, 1);
/// Reserved
pub const RESERVE_0_153: EfuseField = EfuseField::new(0, 4, 153, 7);
/// Secure version for anti-rollback
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 5, 160, 16);
/// Disable fast verify on wake when secure boot is enabled
pub const SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField = EfuseField::new(0, 5, 176, 1);
/// Enable hysteresis on corresponding pad
pub const HYS_EN_PAD: EfuseField = EfuseField::new(0, 5, 177, 1);
/// Reserved
pub const RESERVE_0_178: EfuseField = EfuseField::new(0, 5, 178, 14);
/// Reserved
pub const RESERVE_0_192: EfuseField = EfuseField::new(0, 6, 192, 2);
/// Select DCDC vset from efuse_dcdc_vset
pub const DCDC_VSET_EN: EfuseField = EfuseField::new(0, 6, 194, 1);
/// Disable watchdog
pub const DIS_WDT: EfuseField = EfuseField::new(0, 6, 195, 1);
/// Disable super-watchdog
pub const DIS_SWD: EfuseField = EfuseField::new(0, 6, 196, 1);
/// Reserved
pub const RESERVE_0_197: EfuseField = EfuseField::new(0, 6, 197, 6);
/// Enable secure boot using SHA-384
pub const SECURE_BOOT_SHA384_EN: EfuseField = EfuseField::new(0, 6, 203, 1);
/// Anti-rollback secure version for 2nd-stage bootloader
pub const BOOTLOADER_ANTI_ROLLBACK_SECURE_VERSION: EfuseField = EfuseField::new(0, 6, 204, 4);
/// Enable anti-rollback check for 2nd-stage bootloader
pub const BOOTLOADER_ANTI_ROLLBACK_EN: EfuseField = EfuseField::new(0, 6, 208, 1);
/// Enable anti-rollback SECURE_VERSION update from ROM bootloader
pub const BOOTLOADER_ANTI_ROLLBACK_UPDATE_IN_ROM: EfuseField = EfuseField::new(0, 6, 209, 1);
/// Starting flash sector of recovery bootloader (0 or 0xFFF = disabled)
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR: EfuseField = EfuseField::new(0, 6, 210, 12);
/// Enable RMA function in download mode (01/10 = enabled, 00/11 = disabled)
pub const RMA_ENA: EfuseField = EfuseField::new(0, 6, 222, 2);
/// Number of times the RMA session has been entered
pub const RMA_SESSION_COUNTER: EfuseField = EfuseField::new(0, 7, 224, 3);
/// RMA NONCE enable (x0 = no NONCE, 1x = KM-generated NONCE)
pub const RMA_NONCE_ENA: EfuseField = EfuseField::new(0, 7, 227, 2);
/// Use HUK_info as CHIP_info source in RMA (0 = UNIQ_id)
pub const RMA_CHIP_INFO_SOURCE: EfuseField = EfuseField::new(0, 7, 229, 1);
/// Disable FAST_VEF in RMA session
pub const RMA_DISABLE_FAST_VEF: EfuseField = EfuseField::new(0, 7, 230, 1);
/// Enable PVT power glitch monitor 0
pub const PVT_0_GLITCH_EN: EfuseField = EfuseField::new(0, 7, 231, 1);
/// Glitch mode for PVT monitor 0
pub const PVT_0_GLITCH_MODE: EfuseField = EfuseField::new(0, 7, 232, 2);
/// Enable PVT power glitch monitor 1
pub const PVT_1_GLITCH_EN: EfuseField = EfuseField::new(0, 7, 234, 1);
/// Glitch mode for PVT monitor 1
pub const PVT_1_GLITCH_MODE: EfuseField = EfuseField::new(0, 7, 235, 2);
/// Flash power select (1 = 3.3V, 0 = 1.8V)
pub const PMU_FLASH_POWER_SEL: EfuseField = EfuseField::new(0, 7, 237, 1);
/// Validate PMU_FLASH_POWER_SEL
pub const PMU_FLASH_POWER_SEL_EN: EfuseField = EfuseField::new(0, 7, 238, 1);
/// Enable power glitch detection
pub const POWER_GLITCH_EN: EfuseField = EfuseField::new(0, 7, 239, 4);
/// Enable XTS-AES shadow core countermeasure against fault injection
pub const ENA_XTS_SHADOW: EfuseField = EfuseField::new(0, 7, 243, 1);
/// Enable ciphertext scrambler for external memory
pub const ENA_SPI_BOOT_CRYPT_SCRAMBLER: EfuseField = EfuseField::new(0, 7, 244, 1);
/// Select crypto peripheral for re-enabling JTAG (0 = RMA, 1 = HMAC)
pub const RE_ENABLE_JTAG_SOURCE: EfuseField = EfuseField::new(0, 7, 245, 1);
/// Reserved
pub const RESERVE_0_246: EfuseField = EfuseField::new(0, 7, 246, 10);
/// Reserved
pub const REPEAT7_RSVD: EfuseField = EfuseField::new(0, 8, 256, 16);
/// Reserved
pub const RESERVE_0_272: EfuseField = EfuseField::new(0, 8, 272, 16);
/// `[MAC_FACTORY]` MAC address (bits 0–31)
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// `[MAC_FACTORY]` MAC address (bits 32–47)
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// Extended MAC address bits
pub const MAC_EXT: EfuseField = EfuseField::new(1, 1, 48, 16);
/// Reserved
pub const MAC_RESERVED_0: EfuseField = EfuseField::new(1, 2, 64, 14);
/// Reserved
pub const MAC_RESERVED_1: EfuseField = EfuseField::new(1, 2, 78, 18);
/// Reserved
pub const MAC_RESERVED_2: EfuseField = EfuseField::new(1, 3, 96, 18);
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
/// Reserved
pub const RESERVED_1_136: EfuseField = EfuseField::new(1, 4, 136, 24);
/// Third 32-bit word of zeroth system data block
pub const SYS_DATA_PART0_2: EfuseField = EfuseField::new(1, 5, 160, 32);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// First-part system data word 4
pub const SYS_DATA_PART1_4: EfuseField = EfuseField::new(2, 4, 128, 32);
/// First-part system data word 5
pub const SYS_DATA_PART1_5: EfuseField = EfuseField::new(2, 5, 160, 32);
/// First-part system data word 6
pub const SYS_DATA_PART1_6: EfuseField = EfuseField::new(2, 6, 192, 32);
/// First-part system data word 7
pub const SYS_DATA_PART1_7: EfuseField = EfuseField::new(2, 7, 224, 32);
/// `[BLOCK_USR_DATA]` User data
pub const BLOCK_USR_DATA: EfuseField = EfuseField::new(3, 0, 0, 192);
/// Reserved
pub const RESERVED_3_192: EfuseField = EfuseField::new(3, 6, 192, 8);
/// `[MAC_CUSTOM CUSTOM_MAC]` Custom MAC
pub const CUSTOM_MAC: EfuseField = EfuseField::new(3, 6, 200, 48);
/// Reserved
pub const RESERVED_3_248: EfuseField = EfuseField::new(3, 7, 248, 8);
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
pub const BLOCK_SYS_DATA2: EfuseField = EfuseField::new(9, 0, 0, 32);
/// Second-part system data word 1
pub const SYS_DATA_PART2_1: EfuseField = EfuseField::new(9, 1, 32, 32);
/// Second-part system data word 2
pub const SYS_DATA_PART2_2: EfuseField = EfuseField::new(9, 2, 64, 32);
/// Second-part system data word 3
pub const SYS_DATA_PART2_3: EfuseField = EfuseField::new(9, 3, 96, 32);
/// Second-part system data word 4
pub const SYS_DATA_PART2_4: EfuseField = EfuseField::new(9, 4, 128, 32);
/// Second-part system data word 5
pub const SYS_DATA_PART2_5: EfuseField = EfuseField::new(9, 5, 160, 32);
/// Enable USB device D+/D- pin swap
pub const USB_DEVICE_EXCHG_PINS: EfuseField = EfuseField::new(9, 6, 192, 1);
/// USB device single-end input high threshold (1.76–2V, 80mV steps)
pub const USB_DEVICE_DREFH: EfuseField = EfuseField::new(9, 6, 193, 2);
/// USB device single-end input low threshold (0.8–1.04V, 80mV steps)
pub const USB_DEVICE_DREFL: EfuseField = EfuseField::new(9, 6, 195, 2);
/// Reserved
pub const RESERVE_9_197: EfuseField = EfuseField::new(9, 6, 197, 12);
/// PVT glitch monitor 0 cell select
pub const PVT_0_CELL_SELECT: EfuseField = EfuseField::new(9, 6, 209, 7);
/// PVT glitch monitor 1 cell select
pub const PVT_1_CELL_SELECT: EfuseField = EfuseField::new(9, 6, 216, 7);
/// Reserved
pub const RESERVE_9_223: EfuseField = EfuseField::new(9, 6, 223, 1);
/// PVT glitch monitor 0 threshold
pub const PVT_0_LIMIT: EfuseField = EfuseField::new(9, 7, 224, 16);
/// PVT glitch monitor 1 threshold
pub const PVT_1_LIMIT: EfuseField = EfuseField::new(9, 7, 240, 16);

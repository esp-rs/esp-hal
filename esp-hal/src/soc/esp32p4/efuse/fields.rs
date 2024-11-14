//! eFuse fields for the ESP32-P4.
//!
//! This file was automatically generated, please do not edit it manually!
//!
//! For information on how to regenerate these files, please refer to the
//! `xtask` package's `README.md` file.
//!
//! Generated on:   2024-10-02
//! ESP-IDF Commit: 46acfdce

use super::EfuseBlock;
use crate::soc::efuse_field::EfuseField;

/// `[]` Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 32);
/// `[]` wr_dis of RD_DIS
pub const WR_DIS_RD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 1);
/// `[]` wr_dis of KM_RND_SWITCH_CYCLE
pub const WR_DIS_KM_RND_SWITCH_CYCLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of KM_DEPLOY_ONLY_ONCE
pub const WR_DIS_KM_DEPLOY_ONLY_ONCE: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of FORCE_USE_KEY_MANAGER_KEY
pub const WR_DIS_FORCE_USE_KEY_MANAGER_KEY: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of FORCE_DISABLE_SW_INIT_KEY
pub const WR_DIS_FORCE_DISABLE_SW_INIT_KEY: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of XTS_KEY_LENGTH_256
pub const WR_DIS_XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of LOCK_KM_KEY
pub const WR_DIS_LOCK_KM_KEY: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of KM_DISABLE_DEPLOY_MODE
pub const WR_DIS_KM_DISABLE_DEPLOY_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of DIS_USB_JTAG
pub const WR_DIS_DIS_USB_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_FORCE_DOWNLOAD
pub const WR_DIS_DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of SPI_DOWNLOAD_MSPI_DIS
pub const WR_DIS_SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_TWAI
pub const WR_DIS_DIS_TWAI: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of JTAG_SEL_ENABLE
pub const WR_DIS_JTAG_SEL_ENABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_PAD_JTAG
pub const WR_DIS_DIS_PAD_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_MANUAL_ENCRYPT
pub const WR_DIS_DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of WDT_DELAY_SEL
pub const WR_DIS_WDT_DELAY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of HYS_EN_PAD
pub const WR_DIS_HYS_EN_PAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of PXA0_TIEH_SEL_0
pub const WR_DIS_PXA0_TIEH_SEL_0: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of PXA0_TIEH_SEL_1
pub const WR_DIS_PXA0_TIEH_SEL_1: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of PXA0_TIEH_SEL_2
pub const WR_DIS_PXA0_TIEH_SEL_2: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of PXA0_TIEH_SEL_3
pub const WR_DIS_PXA0_TIEH_SEL_3: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_WDT
pub const WR_DIS_DIS_WDT: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_SWD
pub const WR_DIS_DIS_SWD: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of HP_PWR_SRC_SEL
pub const WR_DIS_HP_PWR_SRC_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of SPI_BOOT_CRYPT_CNT
pub const WR_DIS_SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(EfuseBlock::Block0, 4, 1);
/// `[]` wr_dis of SECURE_BOOT_KEY_REVOKE0
pub const WR_DIS_SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(EfuseBlock::Block0, 5, 1);
/// `[]` wr_dis of SECURE_BOOT_KEY_REVOKE1
pub const WR_DIS_SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of SECURE_BOOT_KEY_REVOKE2
pub const WR_DIS_SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(EfuseBlock::Block0, 7, 1);
/// `[WR_DIS.KEY0_PURPOSE]` wr_dis of KEY_PURPOSE_0
pub const WR_DIS_KEY_PURPOSE_0: EfuseField = EfuseField::new(EfuseBlock::Block0, 8, 1);
/// `[WR_DIS.KEY1_PURPOSE]` wr_dis of KEY_PURPOSE_1
pub const WR_DIS_KEY_PURPOSE_1: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[WR_DIS.KEY2_PURPOSE]` wr_dis of KEY_PURPOSE_2
pub const WR_DIS_KEY_PURPOSE_2: EfuseField = EfuseField::new(EfuseBlock::Block0, 10, 1);
/// `[WR_DIS.KEY3_PURPOSE]` wr_dis of KEY_PURPOSE_3
pub const WR_DIS_KEY_PURPOSE_3: EfuseField = EfuseField::new(EfuseBlock::Block0, 11, 1);
/// `[WR_DIS.KEY4_PURPOSE]` wr_dis of KEY_PURPOSE_4
pub const WR_DIS_KEY_PURPOSE_4: EfuseField = EfuseField::new(EfuseBlock::Block0, 12, 1);
/// `[WR_DIS.KEY5_PURPOSE]` wr_dis of KEY_PURPOSE_5
pub const WR_DIS_KEY_PURPOSE_5: EfuseField = EfuseField::new(EfuseBlock::Block0, 13, 1);
/// `[]` wr_dis of SEC_DPA_LEVEL
pub const WR_DIS_SEC_DPA_LEVEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 14, 1);
/// `[]` wr_dis of CRYPT_DPA_ENABLE
pub const WR_DIS_CRYPT_DPA_ENABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 14, 1);
/// `[]` wr_dis of SECURE_BOOT_EN
pub const WR_DIS_SECURE_BOOT_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 15, 1);
/// `[]` wr_dis of SECURE_BOOT_AGGRESSIVE_REVOKE
pub const WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 16, 1);
/// `[]` wr_dis of ECDSA_ENABLE_SOFT_K
pub const WR_DIS_ECDSA_ENABLE_SOFT_K: EfuseField = EfuseField::new(EfuseBlock::Block0, 17, 1);
/// `[]` wr_dis of FLASH_TYPE
pub const WR_DIS_FLASH_TYPE: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of FLASH_PAGE_SIZE
pub const WR_DIS_FLASH_PAGE_SIZE: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of FLASH_ECC_EN
pub const WR_DIS_FLASH_ECC_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_USB_OTG_DOWNLOAD_MODE
pub const WR_DIS_DIS_USB_OTG_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of FLASH_TPUW
pub const WR_DIS_FLASH_TPUW: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_MODE
pub const WR_DIS_DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_DIRECT_BOOT
pub const WR_DIS_DIS_DIRECT_BOOT: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_USB_SERIAL_JTAG_ROM_PRINT
pub const WR_DIS_DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE
pub const WR_DIS_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of ENABLE_SECURITY_DOWNLOAD
pub const WR_DIS_ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of UART_PRINT_CONTROL
pub const WR_DIS_UART_PRINT_CONTROL: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of FORCE_SEND_RESUME
pub const WR_DIS_FORCE_SEND_RESUME: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of SECURE_VERSION
pub const WR_DIS_SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of SECURE_BOOT_DISABLE_FAST_WAKE
pub const WR_DIS_SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of KM_HUK_GEN_STATE
pub const WR_DIS_KM_HUK_GEN_STATE: EfuseField = EfuseField::new(EfuseBlock::Block0, 19, 1);
/// `[]` wr_dis of BLOCK1
pub const WR_DIS_BLK1: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[WR_DIS.MAC_FACTORY]` wr_dis of MAC
pub const WR_DIS_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of WAFER_VERSION_MINOR
pub const WR_DIS_WAFER_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of WAFER_VERSION_MAJOR
pub const WR_DIS_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of DISABLE_WAFER_VERSION_MAJOR
pub const WR_DIS_DISABLE_WAFER_VERSION_MAJOR: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of DISABLE_BLK_VERSION_MAJOR
pub const WR_DIS_DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of BLK_VERSION_MINOR
pub const WR_DIS_BLK_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of BLK_VERSION_MAJOR
pub const WR_DIS_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of PSRAM_CAP
pub const WR_DIS_PSRAM_CAP: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of TEMP
pub const WR_DIS_TEMP: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of PSRAM_VENDOR
pub const WR_DIS_PSRAM_VENDOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of PKG_VERSION
pub const WR_DIS_PKG_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of BLOCK2
pub const WR_DIS_SYS_DATA_PART1: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of OPTIONAL_UNIQUE_ID
pub const WR_DIS_OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[WR_DIS.USER_DATA]` wr_dis of BLOCK_USR_DATA
pub const WR_DIS_BLOCK_USR_DATA: EfuseField = EfuseField::new(EfuseBlock::Block0, 22, 1);
/// `[WR_DIS.MAC_CUSTOM WR_DIS.USER_DATA_MAC_CUSTOM]` wr_dis of CUSTOM_MAC
pub const WR_DIS_CUSTOM_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 22, 1);
/// `[WR_DIS.KEY0]` wr_dis of BLOCK_KEY0
pub const WR_DIS_BLOCK_KEY0: EfuseField = EfuseField::new(EfuseBlock::Block0, 23, 1);
/// `[WR_DIS.KEY1]` wr_dis of BLOCK_KEY1
pub const WR_DIS_BLOCK_KEY1: EfuseField = EfuseField::new(EfuseBlock::Block0, 24, 1);
/// `[WR_DIS.KEY2]` wr_dis of BLOCK_KEY2
pub const WR_DIS_BLOCK_KEY2: EfuseField = EfuseField::new(EfuseBlock::Block0, 25, 1);
/// `[WR_DIS.KEY3]` wr_dis of BLOCK_KEY3
pub const WR_DIS_BLOCK_KEY3: EfuseField = EfuseField::new(EfuseBlock::Block0, 26, 1);
/// `[WR_DIS.KEY4]` wr_dis of BLOCK_KEY4
pub const WR_DIS_BLOCK_KEY4: EfuseField = EfuseField::new(EfuseBlock::Block0, 27, 1);
/// `[WR_DIS.KEY5]` wr_dis of BLOCK_KEY5
pub const WR_DIS_BLOCK_KEY5: EfuseField = EfuseField::new(EfuseBlock::Block0, 28, 1);
/// `[WR_DIS.SYS_DATA_PART2]` wr_dis of BLOCK_SYS_DATA2
pub const WR_DIS_BLOCK_SYS_DATA2: EfuseField = EfuseField::new(EfuseBlock::Block0, 29, 1);
/// `[]` wr_dis of USB_DEVICE_EXCHG_PINS
pub const WR_DIS_USB_DEVICE_EXCHG_PINS: EfuseField = EfuseField::new(EfuseBlock::Block0, 30, 1);
/// `[]` wr_dis of USB_OTG11_EXCHG_PINS
pub const WR_DIS_USB_OTG11_EXCHG_PINS: EfuseField = EfuseField::new(EfuseBlock::Block0, 30, 1);
/// `[]` wr_dis of USB_PHY_SEL
pub const WR_DIS_USB_PHY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 30, 1);
/// `[]` wr_dis of SOFT_DIS_JTAG
pub const WR_DIS_SOFT_DIS_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 31, 1);
/// `[]` Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 32, 7);
/// `[RD_DIS.KEY0]` rd_dis of BLOCK_KEY0
pub const RD_DIS_BLOCK_KEY0: EfuseField = EfuseField::new(EfuseBlock::Block0, 32, 1);
/// `[RD_DIS.KEY1]` rd_dis of BLOCK_KEY1
pub const RD_DIS_BLOCK_KEY1: EfuseField = EfuseField::new(EfuseBlock::Block0, 33, 1);
/// `[RD_DIS.KEY2]` rd_dis of BLOCK_KEY2
pub const RD_DIS_BLOCK_KEY2: EfuseField = EfuseField::new(EfuseBlock::Block0, 34, 1);
/// `[RD_DIS.KEY3]` rd_dis of BLOCK_KEY3
pub const RD_DIS_BLOCK_KEY3: EfuseField = EfuseField::new(EfuseBlock::Block0, 35, 1);
/// `[RD_DIS.KEY4]` rd_dis of BLOCK_KEY4
pub const RD_DIS_BLOCK_KEY4: EfuseField = EfuseField::new(EfuseBlock::Block0, 36, 1);
/// `[RD_DIS.KEY5]` rd_dis of BLOCK_KEY5
pub const RD_DIS_BLOCK_KEY5: EfuseField = EfuseField::new(EfuseBlock::Block0, 37, 1);
/// `[RD_DIS.SYS_DATA_PART2]` rd_dis of BLOCK_SYS_DATA2
pub const RD_DIS_BLOCK_SYS_DATA2: EfuseField = EfuseField::new(EfuseBlock::Block0, 38, 1);
/// `[]` Enable usb device exchange pins of D+ and D-
pub const USB_DEVICE_EXCHG_PINS: EfuseField = EfuseField::new(EfuseBlock::Block0, 39, 1);
/// `[]` Enable usb otg11 exchange pins of D+ and D-
pub const USB_OTG11_EXCHG_PINS: EfuseField = EfuseField::new(EfuseBlock::Block0, 40, 1);
/// `[]` Represents whether the function of usb switch to jtag is disabled or
/// enabled. 1: disabled. 0: enabled
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 41, 1);
/// `[]` Represents whether power glitch function is enabled. 1: enabled. 0:
/// disabled
pub const POWERGLITCH_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 42, 1);
/// `[]` Represents whether the function that forces chip into download mode is
/// disabled or enabled. 1: disabled. 0: enabled
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 44, 1);
/// `[]` Set this bit to disable accessing MSPI flash/MSPI ram by SYS AXI matrix
/// during boot_mode_download
pub const SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 45, 1);
/// `[]` Represents whether TWAI function is disabled or enabled. 1: disabled.
/// 0: enabled
pub const DIS_TWAI: EfuseField = EfuseField::new(EfuseBlock::Block0, 46, 1);
/// `[]` Represents whether the selection between usb_to_jtag and pad_to_jtag
/// through strapping gpio15 when both EFUSE_DIS_PAD_JTAG and EFUSE_DIS_USB_JTAG
/// are equal to 0 is enabled or disabled. 1: enabled. 0: disabled
pub const JTAG_SEL_ENABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 47, 1);
/// `[]` Represents whether JTAG is disabled in soft way. Odd number: disabled.
/// Even number: enabled
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 48, 3);
/// `[]` Represents whether JTAG is disabled in the hard way(permanently). 1:
/// disabled. 0: enabled
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 51, 1);
/// `[]` Represents whether flash encrypt function is disabled or enabled(except
/// in SPI boot mode). 1: disabled. 0: enabled
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(EfuseBlock::Block0, 52, 1);
/// `[]` TBD
pub const USB_PHY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 57, 1);
/// `[]` Set this bit to control validation of HUK generate mode. Odd of 1 is
/// invalid; even of 1 is valid
pub const KM_HUK_GEN_STATE: EfuseField = EfuseField::new(EfuseBlock::Block0, 58, 9);
/// `[]` Set bits to control key manager random number switch cycle. 0: control
/// by register. 1: 8 km clk cycles. 2: 16 km cycles. 3: 32 km cycles
pub const KM_RND_SWITCH_CYCLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 67, 2);
/// `[]` Set each bit to control whether corresponding key can only be deployed
/// once. 1 is true; 0 is false. Bit0: ecdsa. Bit1: xts. Bit2: hmac. Bit3: ds
pub const KM_DEPLOY_ONLY_ONCE: EfuseField = EfuseField::new(EfuseBlock::Block0, 69, 4);
/// `[]` Set each bit to control whether corresponding key must come from key
/// manager.. 1 is true; 0 is false. Bit0: ecdsa. Bit1: xts. Bit2: hmac. Bit3:
/// ds
pub const FORCE_USE_KEY_MANAGER_KEY: EfuseField = EfuseField::new(EfuseBlock::Block0, 73, 4);
/// `[]` Set this bit to disable software written init key; and force use
/// efuse_init_key
pub const FORCE_DISABLE_SW_INIT_KEY: EfuseField = EfuseField::new(EfuseBlock::Block0, 77, 1);
/// `[]` Set this bit to configure flash encryption use xts-128 key; else use
/// xts-256 key
pub const XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(EfuseBlock::Block0, 78, 1);
/// `[]` Represents whether RTC watchdog timeout threshold is selected at
/// startup. 1: selected. 0: not selected
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 80, 2);
/// `[]` Enables flash encryption when 1 or 3 bits are set and disables
/// otherwise {0: "Disable"; 1: "Enable"; 3: "Disable"; 7: "Enable"}
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(EfuseBlock::Block0, 82, 3);
/// `[]` Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(EfuseBlock::Block0, 85, 1);
/// `[]` Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(EfuseBlock::Block0, 86, 1);
/// `[]` Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(EfuseBlock::Block0, 87, 1);
/// `[KEY0_PURPOSE]` Represents the purpose of Key0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(EfuseBlock::Block0, 88, 4);
/// `[KEY1_PURPOSE]` Represents the purpose of Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(EfuseBlock::Block0, 92, 4);
/// `[KEY2_PURPOSE]` Represents the purpose of Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(EfuseBlock::Block0, 96, 4);
/// `[KEY3_PURPOSE]` Represents the purpose of Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(EfuseBlock::Block0, 100, 4);
/// `[KEY4_PURPOSE]` Represents the purpose of Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(EfuseBlock::Block0, 104, 4);
/// `[KEY5_PURPOSE]` Represents the purpose of Key5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(EfuseBlock::Block0, 108, 4);
/// `[]` Represents the spa secure level by configuring the clock random divide
/// mode
pub const SEC_DPA_LEVEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 112, 2);
/// `[]` Represents whether hardware random number k is forced used in ESDCA. 1:
/// force used. 0: not force used
pub const ECDSA_ENABLE_SOFT_K: EfuseField = EfuseField::new(EfuseBlock::Block0, 114, 1);
/// `[]` Represents whether anti-dpa attack is enabled. 1:enabled. 0: disabled
pub const CRYPT_DPA_ENABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 115, 1);
/// `[]` Represents whether secure boot is enabled or disabled. 1: enabled. 0:
/// disabled
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 116, 1);
/// `[]` Represents whether revoking aggressive secure boot is enabled or
/// disabled. 1: enabled. 0: disabled
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(EfuseBlock::Block0, 117, 1);
/// `[]` The type of interfaced flash. 0: four data lines; 1: eight data lines
pub const FLASH_TYPE: EfuseField = EfuseField::new(EfuseBlock::Block0, 119, 1);
/// `[]` Set flash page size
pub const FLASH_PAGE_SIZE: EfuseField = EfuseField::new(EfuseBlock::Block0, 120, 2);
/// `[]` Set this bit to enable ecc for flash boot
pub const FLASH_ECC_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 122, 1);
/// `[]` Set this bit to disable download via USB-OTG
pub const DIS_USB_OTG_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 123, 1);
/// `[]` Represents the flash waiting time after power-up; in unit of ms. When
/// the value less than 15; the waiting time is the programmed value. Otherwise;
/// the waiting time is 2 times the programmed value
pub const FLASH_TPUW: EfuseField = EfuseField::new(EfuseBlock::Block0, 124, 4);
/// `[]` Represents whether Download mode is disabled or enabled. 1: disabled.
/// 0: enabled
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 128, 1);
/// `[]` Represents whether direct boot mode is disabled or enabled. 1:
/// disabled. 0: enabled
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(EfuseBlock::Block0, 129, 1);
/// `[]` Represents whether print from USB-Serial-JTAG is disabled or enabled.
/// 1: disabled. 0: enabled
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(EfuseBlock::Block0, 130, 1);
/// `[]` TBD
pub const LOCK_KM_KEY: EfuseField = EfuseField::new(EfuseBlock::Block0, 131, 1);
/// `[]` Represents whether the USB-Serial-JTAG download function is disabled or
/// enabled. 1: disabled. 0: enabled
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 132, 1);
/// `[]` Represents whether security download is enabled or disabled. 1:
/// enabled. 0: disabled
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 133, 1);
/// `[]` Represents the type of UART printing. 00: force enable printing. 01:
/// enable printing when GPIO8 is reset at low level. 10: enable printing when
/// GPIO8 is reset at high level. 11: force disable printing
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(EfuseBlock::Block0, 134, 2);
/// `[]` Represents whether ROM code is forced to send a resume command during
/// SPI boot. 1: forced. 0:not forced
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(EfuseBlock::Block0, 136, 1);
/// `[]` Represents the version used by ESP-IDF anti-rollback feature
pub const SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 137, 16);
/// `[]` Represents whether FAST VERIFY ON WAKE is disabled or enabled when
/// Secure Boot is enabled. 1: disabled. 0: enabled
pub const SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField = EfuseField::new(EfuseBlock::Block0, 153, 1);
/// `[]` Represents whether the hysteresis function of corresponding PAD is
/// enabled. 1: enabled. 0:disabled
pub const HYS_EN_PAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 154, 1);
/// `[]` Set the dcdc voltage default
pub const DCDC_VSET: EfuseField = EfuseField::new(EfuseBlock::Block0, 155, 5);
/// `[]` TBD
pub const PXA0_TIEH_SEL_0: EfuseField = EfuseField::new(EfuseBlock::Block0, 160, 2);
/// `[]` TBD
pub const PXA0_TIEH_SEL_1: EfuseField = EfuseField::new(EfuseBlock::Block0, 162, 2);
/// `[]` TBD
pub const PXA0_TIEH_SEL_2: EfuseField = EfuseField::new(EfuseBlock::Block0, 164, 2);
/// `[]` TBD
pub const PXA0_TIEH_SEL_3: EfuseField = EfuseField::new(EfuseBlock::Block0, 166, 2);
/// `[]` TBD
pub const KM_DISABLE_DEPLOY_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 168, 4);
/// `[]` HP system power source select. 0:LDO. 1: DCDC
pub const HP_PWR_SRC_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 178, 1);
/// `[]` Select dcdc vset use efuse_dcdc_vset
pub const DCDC_VSET_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 179, 1);
/// `[]` Set this bit to disable watch dog
pub const DIS_WDT: EfuseField = EfuseField::new(EfuseBlock::Block0, 180, 1);
/// `[]` Set this bit to disable super-watchdog
pub const DIS_SWD: EfuseField = EfuseField::new(EfuseBlock::Block0, 181, 1);
/// `[MAC_FACTORY]` MAC address
pub const MAC: EfuseField = EfuseField::new(EfuseBlock::Block1, 0, 48);
/// `[]` Minor chip version
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 64, 4);
/// `[]` Major chip version
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 68, 2);
/// `[]` Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 70, 1);
/// `[]` Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 71, 1);
/// `[]` BLK_VERSION_MINOR of BLOCK2
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 72, 3);
/// `[]` BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 75, 2);
/// `[]` PSRAM capacity
pub const PSRAM_CAP: EfuseField = EfuseField::new(EfuseBlock::Block1, 77, 3);
/// `[]` Operating temperature of the ESP chip
pub const TEMP: EfuseField = EfuseField::new(EfuseBlock::Block1, 80, 2);
/// `[]` PSRAM vendor
pub const PSRAM_VENDOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 82, 2);
/// `[]` Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block1, 84, 3);
/// `[]` Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(EfuseBlock::Block2, 0, 128);
/// `[BLOCK_USR_DATA]` User data
pub const USER_DATA: EfuseField = EfuseField::new(EfuseBlock::Block3, 0, 256);
/// `[MAC_CUSTOM CUSTOM_MAC]` Custom MAC
pub const USER_DATA_MAC_CUSTOM: EfuseField = EfuseField::new(EfuseBlock::Block3, 200, 48);
/// `[BLOCK_KEY0]` Key0 or user data
pub const KEY0: EfuseField = EfuseField::new(EfuseBlock::Block4, 0, 256);
/// `[BLOCK_KEY1]` Key1 or user data
pub const KEY1: EfuseField = EfuseField::new(EfuseBlock::Block5, 0, 256);
/// `[BLOCK_KEY2]` Key2 or user data
pub const KEY2: EfuseField = EfuseField::new(EfuseBlock::Block6, 0, 256);
/// `[BLOCK_KEY3]` Key3 or user data
pub const KEY3: EfuseField = EfuseField::new(EfuseBlock::Block7, 0, 256);
/// `[BLOCK_KEY4]` Key4 or user data
pub const KEY4: EfuseField = EfuseField::new(EfuseBlock::Block8, 0, 256);
/// `[BLOCK_KEY5]` Key5 or user data
pub const KEY5: EfuseField = EfuseField::new(EfuseBlock::Block9, 0, 256);
/// `[BLOCK_SYS_DATA2]` System data part 2 (reserved)
pub const SYS_DATA_PART2: EfuseField = EfuseField::new(EfuseBlock::Block10, 0, 256);

//! eFuse fields for the ESP32-S2.
//!
//! This file was automatically generated, please do not edit it manually!
//!
//! For information on how to regenerate these files, please refer to the
//! `xtask` package's `README.md` file.
//!
//! Generated on:   2024-03-11
//! ESP-IDF Commit: 0de2912f

use super::EfuseBlock;
use crate::soc::efuse_field::EfuseField;

/// `[]` Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 32);
/// `[]` wr_dis of RD_DIS
pub const WR_DIS_RD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 1);
/// `[]` wr_dis of DIS_ICACHE
pub const WR_DIS_DIS_ICACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_DCACHE
pub const WR_DIS_DIS_DCACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_ICACHE
pub const WR_DIS_DIS_DOWNLOAD_ICACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_DCACHE
pub const WR_DIS_DIS_DOWNLOAD_DCACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_FORCE_DOWNLOAD
pub const WR_DIS_DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_USB
pub const WR_DIS_DIS_USB: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[WR_DIS.DIS_CAN]` wr_dis of DIS_TWAI
pub const WR_DIS_DIS_TWAI: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_BOOT_REMAP
pub const WR_DIS_DIS_BOOT_REMAP: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of SOFT_DIS_JTAG
pub const WR_DIS_SOFT_DIS_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of HARD_DIS_JTAG
pub const WR_DIS_HARD_DIS_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_MANUAL_ENCRYPT
pub const WR_DIS_DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of VDD_SPI_XPD
pub const WR_DIS_VDD_SPI_XPD: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of VDD_SPI_TIEH
pub const WR_DIS_VDD_SPI_TIEH: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of VDD_SPI_FORCE
pub const WR_DIS_VDD_SPI_FORCE: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of WDT_DELAY_SEL
pub const WR_DIS_WDT_DELAY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
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
/// `[]` wr_dis of SECURE_BOOT_EN
pub const WR_DIS_SECURE_BOOT_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 15, 1);
/// `[]` wr_dis of SECURE_BOOT_AGGRESSIVE_REVOKE
pub const WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 16, 1);
/// `[]` wr_dis of FLASH_TPUW
pub const WR_DIS_FLASH_TPUW: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_MODE
pub const WR_DIS_DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_LEGACY_SPI_BOOT
pub const WR_DIS_DIS_LEGACY_SPI_BOOT: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of UART_PRINT_CHANNEL
pub const WR_DIS_UART_PRINT_CHANNEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of DIS_USB_DOWNLOAD_MODE
pub const WR_DIS_DIS_USB_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of ENABLE_SECURITY_DOWNLOAD
pub const WR_DIS_ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of UART_PRINT_CONTROL
pub const WR_DIS_UART_PRINT_CONTROL: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of PIN_POWER_SELECTION
pub const WR_DIS_PIN_POWER_SELECTION: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of FLASH_TYPE
pub const WR_DIS_FLASH_TYPE: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of FORCE_SEND_RESUME
pub const WR_DIS_FORCE_SEND_RESUME: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of SECURE_VERSION
pub const WR_DIS_SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` wr_dis of BLOCK1
pub const WR_DIS_BLK1: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[WR_DIS.MAC_FACTORY]` wr_dis of MAC
pub const WR_DIS_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_CLK
pub const WR_DIS_SPI_PAD_CONFIG_CLK: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_Q
pub const WR_DIS_SPI_PAD_CONFIG_Q: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_D
pub const WR_DIS_SPI_PAD_CONFIG_D: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_CS
pub const WR_DIS_SPI_PAD_CONFIG_CS: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_HD
pub const WR_DIS_SPI_PAD_CONFIG_HD: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_WP
pub const WR_DIS_SPI_PAD_CONFIG_WP: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_DQS
pub const WR_DIS_SPI_PAD_CONFIG_DQS: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_D4
pub const WR_DIS_SPI_PAD_CONFIG_D4: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_D5
pub const WR_DIS_SPI_PAD_CONFIG_D5: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_D6
pub const WR_DIS_SPI_PAD_CONFIG_D6: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_D7
pub const WR_DIS_SPI_PAD_CONFIG_D7: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of WAFER_VERSION_MAJOR
pub const WR_DIS_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of WAFER_VERSION_MINOR_HI
pub const WR_DIS_WAFER_VERSION_MINOR_HI: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of FLASH_VERSION
pub const WR_DIS_FLASH_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of BLK_VERSION_MAJOR
pub const WR_DIS_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of PSRAM_VERSION
pub const WR_DIS_PSRAM_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of PKG_VERSION
pub const WR_DIS_PKG_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of WAFER_VERSION_MINOR_LO
pub const WR_DIS_WAFER_VERSION_MINOR_LO: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 1);
/// `[]` wr_dis of BLOCK2
pub const WR_DIS_SYS_DATA_PART1: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of OPTIONAL_UNIQUE_ID
pub const WR_DIS_OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of ADC_CALIB
pub const WR_DIS_ADC_CALIB: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of BLK_VERSION_MINOR
pub const WR_DIS_BLK_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of TEMP_CALIB
pub const WR_DIS_TEMP_CALIB: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A10H
pub const WR_DIS_RTCCALIB_V1IDX_A10H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A11H
pub const WR_DIS_RTCCALIB_V1IDX_A11H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A12H
pub const WR_DIS_RTCCALIB_V1IDX_A12H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A13H
pub const WR_DIS_RTCCALIB_V1IDX_A13H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A20H
pub const WR_DIS_RTCCALIB_V1IDX_A20H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A21H
pub const WR_DIS_RTCCALIB_V1IDX_A21H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A22H
pub const WR_DIS_RTCCALIB_V1IDX_A22H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A23H
pub const WR_DIS_RTCCALIB_V1IDX_A23H: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A10L
pub const WR_DIS_RTCCALIB_V1IDX_A10L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A11L
pub const WR_DIS_RTCCALIB_V1IDX_A11L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A12L
pub const WR_DIS_RTCCALIB_V1IDX_A12L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A13L
pub const WR_DIS_RTCCALIB_V1IDX_A13L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A20L
pub const WR_DIS_RTCCALIB_V1IDX_A20L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A21L
pub const WR_DIS_RTCCALIB_V1IDX_A21L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A22L
pub const WR_DIS_RTCCALIB_V1IDX_A22L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
/// `[]` wr_dis of RTCCALIB_V1IDX_A23L
pub const WR_DIS_RTCCALIB_V1IDX_A23L: EfuseField = EfuseField::new(EfuseBlock::Block0, 21, 1);
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
/// `[]` wr_dis of USB_EXCHG_PINS
pub const WR_DIS_USB_EXCHG_PINS: EfuseField = EfuseField::new(EfuseBlock::Block0, 30, 1);
/// `[WR_DIS.EXT_PHY_ENABLE]` wr_dis of USB_EXT_PHY_ENABLE
pub const WR_DIS_USB_EXT_PHY_ENABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 30, 1);
/// `[]` wr_dis of USB_FORCE_NOPERSIST
pub const WR_DIS_USB_FORCE_NOPERSIST: EfuseField = EfuseField::new(EfuseBlock::Block0, 30, 1);
/// `[]` wr_dis of BLOCK0_VERSION
pub const WR_DIS_BLOCK0_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 30, 1);
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
/// `[]` Set this bit to disable Icache
pub const DIS_ICACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 40, 1);
/// `[]` Set this bit to disable Dcache
pub const DIS_DCACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 41, 1);
/// `[]` Disables Icache when SoC is in Download mode
pub const DIS_DOWNLOAD_ICACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 42, 1);
/// `[]` Disables Dcache when SoC is in Download mode
pub const DIS_DOWNLOAD_DCACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 43, 1);
/// `[]` Set this bit to disable the function that forces chip into download
/// mode
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 44, 1);
/// `[]` Set this bit to disable USB OTG function
pub const DIS_USB: EfuseField = EfuseField::new(EfuseBlock::Block0, 45, 1);
/// `[DIS_CAN]` Set this bit to disable the TWAI Controller function
pub const DIS_TWAI: EfuseField = EfuseField::new(EfuseBlock::Block0, 46, 1);
/// `[]` Disables capability to Remap RAM to ROM address space
pub const DIS_BOOT_REMAP: EfuseField = EfuseField::new(EfuseBlock::Block0, 47, 1);
/// `[]` Software disables JTAG. When software disabled; JTAG can be activated
/// temporarily by HMAC peripheral
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 49, 1);
/// `[]` Hardware disables JTAG permanently
pub const HARD_DIS_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 50, 1);
/// `[]` Disables flash encryption when in download boot modes
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(EfuseBlock::Block0, 51, 1);
/// `[]` Set this bit to exchange USB D+ and D- pins
pub const USB_EXCHG_PINS: EfuseField = EfuseField::new(EfuseBlock::Block0, 56, 1);
/// `[EXT_PHY_ENABLE]` Set this bit to enable external USB PHY
pub const USB_EXT_PHY_ENABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 57, 1);
/// `[]` If set; forces USB BVALID to 1
pub const USB_FORCE_NOPERSIST: EfuseField = EfuseField::new(EfuseBlock::Block0, 58, 1);
/// `[]` BLOCK0 efuse version
pub const BLOCK0_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 59, 2);
/// `[]` If VDD_SPI_FORCE is 1; this value determines if the VDD_SPI regulator
/// is powered on
pub const VDD_SPI_XPD: EfuseField = EfuseField::new(EfuseBlock::Block0, 68, 1);
/// `[]` If VDD_SPI_FORCE is 1; determines VDD_SPI voltage {0: "VDD_SPI connects
/// to 1.8 V LDO"; 1: "VDD_SPI connects to VDD3P3_RTC_IO"}
pub const VDD_SPI_TIEH: EfuseField = EfuseField::new(EfuseBlock::Block0, 69, 1);
/// `[]` Set this bit to use XPD_VDD_PSI_REG and VDD_SPI_TIEH to configure
/// VDD_SPI LDO
pub const VDD_SPI_FORCE: EfuseField = EfuseField::new(EfuseBlock::Block0, 70, 1);
/// `[]` RTC watchdog timeout threshold; in unit of slow clock cycle {0:
/// "40000"; 1: "80000"; 2: "160000"; 3: "320000"}
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 80, 2);
/// `[]` Enables flash encryption when 1 or 3 bits are set and disabled
/// otherwise {0: "Disable"; 1: "Enable"; 3: "Disable"; 7: "Enable"}
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(EfuseBlock::Block0, 82, 3);
/// `[]` Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(EfuseBlock::Block0, 85, 1);
/// `[]` Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(EfuseBlock::Block0, 86, 1);
/// `[]` Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(EfuseBlock::Block0, 87, 1);
/// `[KEY0_PURPOSE]` Purpose of KEY0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(EfuseBlock::Block0, 88, 4);
/// `[KEY1_PURPOSE]` Purpose of KEY1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(EfuseBlock::Block0, 92, 4);
/// `[KEY2_PURPOSE]` Purpose of KEY2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(EfuseBlock::Block0, 96, 4);
/// `[KEY3_PURPOSE]` Purpose of KEY3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(EfuseBlock::Block0, 100, 4);
/// `[KEY4_PURPOSE]` Purpose of KEY4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(EfuseBlock::Block0, 104, 4);
/// `[KEY5_PURPOSE]` Purpose of KEY5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(EfuseBlock::Block0, 108, 4);
/// `[]` Set this bit to enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 116, 1);
/// `[]` Set this bit to enable aggressive secure boot key revocation mode
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(EfuseBlock::Block0, 117, 1);
/// `[]` Configures flash startup delay after SoC power-up; in unit of (ms/2).
/// When the value is 15; delay is 7.5 ms
pub const FLASH_TPUW: EfuseField = EfuseField::new(EfuseBlock::Block0, 124, 4);
/// `[]` Set this bit to disable all download boot modes
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 128, 1);
/// `[]` Set this bit to disable Legacy SPI boot mode
pub const DIS_LEGACY_SPI_BOOT: EfuseField = EfuseField::new(EfuseBlock::Block0, 129, 1);
/// `[]` Selects the default UART for printing boot messages {0: "UART0"; 1:
/// "UART1"}
pub const UART_PRINT_CHANNEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 130, 1);
/// `[]` Set this bit to disable use of USB OTG in UART download boot mode
pub const DIS_USB_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 132, 1);
/// `[]` Set this bit to enable secure UART download mode (read/write flash
/// only)
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 133, 1);
/// `[]` Set the default UART boot message output mode {0: "Enable"; 1: "Enable when GPIO46 is low at reset"; 2: "Enable when GPIO46 is high at reset"; 3: "Disable"}
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(EfuseBlock::Block0, 134, 2);
/// `[]` Set default power supply for GPIO33-GPIO37; set when SPI flash is
/// initialized {0: "VDD3P3_CPU"; 1: "VDD_SPI"}
pub const PIN_POWER_SELECTION: EfuseField = EfuseField::new(EfuseBlock::Block0, 136, 1);
/// `[]` SPI flash type {0: "4 data lines"; 1: "8 data lines"}
pub const FLASH_TYPE: EfuseField = EfuseField::new(EfuseBlock::Block0, 137, 1);
/// `[]` If set; forces ROM code to send an SPI flash resume command during SPI
/// boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(EfuseBlock::Block0, 138, 1);
/// `[]` Secure version (used by ESP-IDF anti-rollback feature)
pub const SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 139, 16);
/// `[]` Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 160, 1);
/// `[]` Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 161, 1);
/// `[MAC_FACTORY]` MAC address
pub const MAC: EfuseField = EfuseField::new(EfuseBlock::Block1, 0, 48);
/// `[]` SPI_PAD_configure CLK
pub const SPI_PAD_CONFIG_CLK: EfuseField = EfuseField::new(EfuseBlock::Block1, 48, 6);
/// `[]` SPI_PAD_configure Q(D1)
pub const SPI_PAD_CONFIG_Q: EfuseField = EfuseField::new(EfuseBlock::Block1, 54, 6);
/// `[]` SPI_PAD_configure D(D0)
pub const SPI_PAD_CONFIG_D: EfuseField = EfuseField::new(EfuseBlock::Block1, 60, 6);
/// `[]` SPI_PAD_configure CS
pub const SPI_PAD_CONFIG_CS: EfuseField = EfuseField::new(EfuseBlock::Block1, 66, 6);
/// `[]` SPI_PAD_configure HD(D3)
pub const SPI_PAD_CONFIG_HD: EfuseField = EfuseField::new(EfuseBlock::Block1, 72, 6);
/// `[]` SPI_PAD_configure WP(D2)
pub const SPI_PAD_CONFIG_WP: EfuseField = EfuseField::new(EfuseBlock::Block1, 78, 6);
/// `[]` SPI_PAD_configure DQS
pub const SPI_PAD_CONFIG_DQS: EfuseField = EfuseField::new(EfuseBlock::Block1, 84, 6);
/// `[]` SPI_PAD_configure D4
pub const SPI_PAD_CONFIG_D4: EfuseField = EfuseField::new(EfuseBlock::Block1, 90, 6);
/// `[]` SPI_PAD_configure D5
pub const SPI_PAD_CONFIG_D5: EfuseField = EfuseField::new(EfuseBlock::Block1, 96, 6);
/// `[]` SPI_PAD_configure D6
pub const SPI_PAD_CONFIG_D6: EfuseField = EfuseField::new(EfuseBlock::Block1, 102, 6);
/// `[]` SPI_PAD_configure D7
pub const SPI_PAD_CONFIG_D7: EfuseField = EfuseField::new(EfuseBlock::Block1, 108, 6);
/// `[]` WAFER_VERSION_MAJOR
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 114, 2);
/// `[]` WAFER_VERSION_MINOR most significant bit
pub const WAFER_VERSION_MINOR_HI: EfuseField = EfuseField::new(EfuseBlock::Block1, 116, 1);
/// `[]` Flash version
pub const FLASH_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block1, 117, 4);
/// `[]` BLK_VERSION_MAJOR
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block1, 121, 2);
/// `[]` PSRAM version
pub const PSRAM_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block1, 124, 4);
/// `[]` Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block1, 128, 4);
/// `[]` WAFER_VERSION_MINOR least significant bits
pub const WAFER_VERSION_MINOR_LO: EfuseField = EfuseField::new(EfuseBlock::Block1, 132, 3);
/// `[]` Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(EfuseBlock::Block2, 0, 128);
/// `[]` 4 bit of ADC calibration
pub const ADC_CALIB: EfuseField = EfuseField::new(EfuseBlock::Block2, 128, 4);
/// `[]` BLK_VERSION_MINOR of BLOCK2 {0: "No calib"; 1: "ADC calib V1"; 2: "ADC
/// calib V2"}
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block2, 132, 3);
/// `[]` Temperature calibration data
pub const TEMP_CALIB: EfuseField = EfuseField::new(EfuseBlock::Block2, 135, 9);
/// `[]`
pub const RTCCALIB_V1IDX_A10H: EfuseField = EfuseField::new(EfuseBlock::Block2, 144, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A11H: EfuseField = EfuseField::new(EfuseBlock::Block2, 152, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A12H: EfuseField = EfuseField::new(EfuseBlock::Block2, 160, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A13H: EfuseField = EfuseField::new(EfuseBlock::Block2, 168, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A20H: EfuseField = EfuseField::new(EfuseBlock::Block2, 176, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A21H: EfuseField = EfuseField::new(EfuseBlock::Block2, 184, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A22H: EfuseField = EfuseField::new(EfuseBlock::Block2, 192, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A23H: EfuseField = EfuseField::new(EfuseBlock::Block2, 200, 8);
/// `[]`
pub const RTCCALIB_V1IDX_A10L: EfuseField = EfuseField::new(EfuseBlock::Block2, 208, 6);
/// `[]`
pub const RTCCALIB_V1IDX_A11L: EfuseField = EfuseField::new(EfuseBlock::Block2, 214, 6);
/// `[]`
pub const RTCCALIB_V1IDX_A12L: EfuseField = EfuseField::new(EfuseBlock::Block2, 220, 6);
/// `[]`
pub const RTCCALIB_V1IDX_A13L: EfuseField = EfuseField::new(EfuseBlock::Block2, 226, 6);
/// `[]`
pub const RTCCALIB_V1IDX_A20L: EfuseField = EfuseField::new(EfuseBlock::Block2, 232, 6);
/// `[]`
pub const RTCCALIB_V1IDX_A21L: EfuseField = EfuseField::new(EfuseBlock::Block2, 238, 6);
/// `[]`
pub const RTCCALIB_V1IDX_A22L: EfuseField = EfuseField::new(EfuseBlock::Block2, 244, 6);
/// `[]`
pub const RTCCALIB_V1IDX_A23L: EfuseField = EfuseField::new(EfuseBlock::Block2, 250, 6);
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

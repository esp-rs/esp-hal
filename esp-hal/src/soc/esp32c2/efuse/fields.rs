//! eFuse fields for the ESP32-C2.
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
pub const WR_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 8);
/// `[]` wr_dis of RD_DIS
pub const WR_DIS_RD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 1);
/// `[]` wr_dis of WDT_DELAY_SEL
pub const WR_DIS_WDT_DELAY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of DIS_PAD_JTAG
pub const WR_DIS_DIS_PAD_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_ICACHE
pub const WR_DIS_DIS_DOWNLOAD_ICACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_MANUAL_ENCRYPT
pub const WR_DIS_DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of SPI_BOOT_CRYPT_CNT
pub const WR_DIS_SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of XTS_KEY_LENGTH_256
pub const WR_DIS_XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of SECURE_BOOT_EN
pub const WR_DIS_SECURE_BOOT_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of UART_PRINT_CONTROL
pub const WR_DIS_UART_PRINT_CONTROL: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of FORCE_SEND_RESUME
pub const WR_DIS_FORCE_SEND_RESUME: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of DIS_DOWNLOAD_MODE
pub const WR_DIS_DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of DIS_DIRECT_BOOT
pub const WR_DIS_DIS_DIRECT_BOOT: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of ENABLE_SECURITY_DOWNLOAD
pub const WR_DIS_ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of FLASH_TPUW
pub const WR_DIS_FLASH_TPUW: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of SECURE_VERSION
pub const WR_DIS_SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 4, 1);
/// `[WR_DIS.ENABLE_CUSTOM_MAC]` wr_dis of CUSTOM_MAC_USED
pub const WR_DIS_CUSTOM_MAC_USED: EfuseField = EfuseField::new(EfuseBlock::Block0, 4, 1);
/// `[]` wr_dis of DISABLE_WAFER_VERSION_MAJOR
pub const WR_DIS_DISABLE_WAFER_VERSION_MAJOR: EfuseField =
    EfuseField::new(EfuseBlock::Block0, 4, 1);
/// `[]` wr_dis of DISABLE_BLK_VERSION_MAJOR
pub const WR_DIS_DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 4, 1);
/// `[WR_DIS.MAC_CUSTOM WR_DIS.USER_DATA_MAC_CUSTOM]` wr_dis of CUSTOM_MAC
pub const WR_DIS_CUSTOM_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 5, 1);
/// `[WR_DIS.MAC_FACTORY]` wr_dis of MAC
pub const WR_DIS_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of WAFER_VERSION_MINOR
pub const WR_DIS_WAFER_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of WAFER_VERSION_MAJOR
pub const WR_DIS_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of PKG_VERSION
pub const WR_DIS_PKG_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of BLK_VERSION_MINOR
pub const WR_DIS_BLK_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of BLK_VERSION_MAJOR
pub const WR_DIS_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of OCODE
pub const WR_DIS_OCODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of TEMP_CALIB
pub const WR_DIS_TEMP_CALIB: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of ADC1_INIT_CODE_ATTEN0
pub const WR_DIS_ADC1_INIT_CODE_ATTEN0: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of ADC1_INIT_CODE_ATTEN3
pub const WR_DIS_ADC1_INIT_CODE_ATTEN3: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of ADC1_CAL_VOL_ATTEN0
pub const WR_DIS_ADC1_CAL_VOL_ATTEN0: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of ADC1_CAL_VOL_ATTEN3
pub const WR_DIS_ADC1_CAL_VOL_ATTEN3: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of DIG_DBIAS_HVT
pub const WR_DIS_DIG_DBIAS_HVT: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of DIG_LDO_SLP_DBIAS2
pub const WR_DIS_DIG_LDO_SLP_DBIAS2: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of DIG_LDO_SLP_DBIAS26
pub const WR_DIS_DIG_LDO_SLP_DBIAS26: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of DIG_LDO_ACT_DBIAS26
pub const WR_DIS_DIG_LDO_ACT_DBIAS26: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of DIG_LDO_ACT_STEPD10
pub const WR_DIS_DIG_LDO_ACT_STEPD10: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of RTC_LDO_SLP_DBIAS13
pub const WR_DIS_RTC_LDO_SLP_DBIAS13: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of RTC_LDO_SLP_DBIAS29
pub const WR_DIS_RTC_LDO_SLP_DBIAS29: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of RTC_LDO_SLP_DBIAS31
pub const WR_DIS_RTC_LDO_SLP_DBIAS31: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of RTC_LDO_ACT_DBIAS31
pub const WR_DIS_RTC_LDO_ACT_DBIAS31: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of RTC_LDO_ACT_DBIAS13
pub const WR_DIS_RTC_LDO_ACT_DBIAS13: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of ADC_CALIBRATION_3
pub const WR_DIS_ADC_CALIBRATION_3: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[WR_DIS.KEY0]` wr_dis of BLOCK_KEY0
pub const WR_DIS_BLOCK_KEY0: EfuseField = EfuseField::new(EfuseBlock::Block0, 7, 1);
/// `[]` Disable reading from BlOCK3
pub const RD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 32, 2);
/// `[]` Read protection for EFUSE_BLK3. KEY0
pub const RD_DIS_KEY0: EfuseField = EfuseField::new(EfuseBlock::Block0, 32, 2);
/// `[]` Read protection for EFUSE_BLK3. KEY0 lower 128-bit key
pub const RD_DIS_KEY0_LOW: EfuseField = EfuseField::new(EfuseBlock::Block0, 32, 1);
/// `[]` Read protection for EFUSE_BLK3. KEY0 higher 128-bit key
pub const RD_DIS_KEY0_HI: EfuseField = EfuseField::new(EfuseBlock::Block0, 33, 1);
/// `[]` RTC watchdog timeout threshold; in unit of slow clock cycle {0:
/// "40000"; 1: "80000"; 2: "160000"; 3: "320000"}
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(EfuseBlock::Block0, 34, 2);
/// `[]` Set this bit to disable pad jtag
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(EfuseBlock::Block0, 36, 1);
/// `[]` The bit be set to disable icache in download mode
pub const DIS_DOWNLOAD_ICACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 37, 1);
/// `[]` The bit be set to disable manual encryption
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(EfuseBlock::Block0, 38, 1);
/// `[]` Enables flash encryption when 1 or 3 bits are set and disables
/// otherwise {0: "Disable"; 1: "Enable"; 3: "Disable"; 7: "Enable"}
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(EfuseBlock::Block0, 39, 3);
/// `[]` Flash encryption key length {0: "128 bits key"; 1: "256 bits key"}
pub const XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(EfuseBlock::Block0, 42, 1);
/// `[]` Set the default UARTboot message output mode {0: "Enable"; 1: "Enable
/// when GPIO8 is low at reset"; 2: "Enable when GPIO8 is high at reset"; 3:
/// "Disable"}
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(EfuseBlock::Block0, 43, 2);
/// `[]` Set this bit to force ROM code to send a resume command during SPI boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(EfuseBlock::Block0, 45, 1);
/// `[]` Set this bit to disable download mode (boot_mode`[3:0]` = 0; 1; 2; 4;
/// 5; 6; 7)
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(EfuseBlock::Block0, 46, 1);
/// `[]` This bit set means disable direct_boot mode
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(EfuseBlock::Block0, 47, 1);
/// `[]` Set this bit to enable secure UART download mode
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(EfuseBlock::Block0, 48, 1);
/// `[]` Configures flash waiting time after power-up; in unit of ms. If the
/// value is less than 15; the waiting time is the configurable value.
/// Otherwise; the waiting time is twice the configurable value
pub const FLASH_TPUW: EfuseField = EfuseField::new(EfuseBlock::Block0, 49, 4);
/// `[]` The bit be set to enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(EfuseBlock::Block0, 53, 1);
/// `[]` Secure version for anti-rollback
pub const SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 54, 4);
/// `[ENABLE_CUSTOM_MAC]` True if MAC_CUSTOM is burned
pub const CUSTOM_MAC_USED: EfuseField = EfuseField::new(EfuseBlock::Block0, 58, 1);
/// `[]` Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 59, 1);
/// `[]` Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 60, 1);
/// `[]` User data block
pub const USER_DATA: EfuseField = EfuseField::new(EfuseBlock::Block1, 0, 88);
/// `[MAC_CUSTOM CUSTOM_MAC]` Custom MAC address
pub const USER_DATA_MAC_CUSTOM: EfuseField = EfuseField::new(EfuseBlock::Block1, 0, 48);
/// `[MAC_FACTORY]` MAC address
pub const MAC: EfuseField = EfuseField::new(EfuseBlock::Block2, 0, 48);
/// `[]` WAFER_VERSION_MINOR
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block2, 48, 4);
/// `[]` WAFER_VERSION_MAJOR
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block2, 52, 2);
/// `[]` EFUSE_PKG_VERSION
pub const PKG_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block2, 54, 3);
/// `[]` Minor version of BLOCK2 {0: "No calib"; 1: "With calib"}
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block2, 57, 3);
/// `[]` Major version of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(EfuseBlock::Block2, 60, 2);
/// `[]` OCode
pub const OCODE: EfuseField = EfuseField::new(EfuseBlock::Block2, 62, 7);
/// `[]` Temperature calibration data
pub const TEMP_CALIB: EfuseField = EfuseField::new(EfuseBlock::Block2, 69, 9);
/// `[]` ADC1 init code at atten0
pub const ADC1_INIT_CODE_ATTEN0: EfuseField = EfuseField::new(EfuseBlock::Block2, 78, 8);
/// `[]` ADC1 init code at atten3
pub const ADC1_INIT_CODE_ATTEN3: EfuseField = EfuseField::new(EfuseBlock::Block2, 86, 5);
/// `[]` ADC1 calibration voltage at atten0
pub const ADC1_CAL_VOL_ATTEN0: EfuseField = EfuseField::new(EfuseBlock::Block2, 91, 8);
/// `[]` ADC1 calibration voltage at atten3
pub const ADC1_CAL_VOL_ATTEN3: EfuseField = EfuseField::new(EfuseBlock::Block2, 99, 6);
/// `[]` BLOCK2 digital dbias when hvt
pub const DIG_DBIAS_HVT: EfuseField = EfuseField::new(EfuseBlock::Block2, 105, 5);
/// `[]` BLOCK2 DIG_LDO_DBG0_DBIAS2
pub const DIG_LDO_SLP_DBIAS2: EfuseField = EfuseField::new(EfuseBlock::Block2, 110, 7);
/// `[]` BLOCK2 DIG_LDO_DBG0_DBIAS26
pub const DIG_LDO_SLP_DBIAS26: EfuseField = EfuseField::new(EfuseBlock::Block2, 117, 8);
/// `[]` BLOCK2 DIG_LDO_ACT_DBIAS26
pub const DIG_LDO_ACT_DBIAS26: EfuseField = EfuseField::new(EfuseBlock::Block2, 125, 6);
/// `[]` BLOCK2 DIG_LDO_ACT_STEPD10
pub const DIG_LDO_ACT_STEPD10: EfuseField = EfuseField::new(EfuseBlock::Block2, 131, 4);
/// `[]` BLOCK2 DIG_LDO_SLP_DBIAS13
pub const RTC_LDO_SLP_DBIAS13: EfuseField = EfuseField::new(EfuseBlock::Block2, 135, 7);
/// `[]` BLOCK2 DIG_LDO_SLP_DBIAS29
pub const RTC_LDO_SLP_DBIAS29: EfuseField = EfuseField::new(EfuseBlock::Block2, 142, 9);
/// `[]` BLOCK2 DIG_LDO_SLP_DBIAS31
pub const RTC_LDO_SLP_DBIAS31: EfuseField = EfuseField::new(EfuseBlock::Block2, 151, 6);
/// `[]` BLOCK2 DIG_LDO_ACT_DBIAS31
pub const RTC_LDO_ACT_DBIAS31: EfuseField = EfuseField::new(EfuseBlock::Block2, 157, 6);
/// `[]` BLOCK2 DIG_LDO_ACT_DBIAS13
pub const RTC_LDO_ACT_DBIAS13: EfuseField = EfuseField::new(EfuseBlock::Block2, 163, 8);
/// `[]` Store the bit `[86:96]` of ADC calibration data
pub const ADC_CALIBRATION_3: EfuseField = EfuseField::new(EfuseBlock::Block2, 192, 11);
/// `[BLOCK_KEY0]` BLOCK_BLOCK_KEY0 - 256-bits. 256-bit key of Flash Encryption
pub const KEY0: EfuseField = EfuseField::new(EfuseBlock::Block3, 0, 256);
/// `[]` 256bit FE key
pub const KEY0_FE_256BIT: EfuseField = EfuseField::new(EfuseBlock::Block3, 0, 256);
/// `[]` 128bit FE key
pub const KEY0_FE_128BIT: EfuseField = EfuseField::new(EfuseBlock::Block3, 0, 128);
/// `[]` 128bit SB key
pub const KEY0_SB_128BIT: EfuseField = EfuseField::new(EfuseBlock::Block3, 128, 128);

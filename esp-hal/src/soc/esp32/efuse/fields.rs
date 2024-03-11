//! eFuse fields for the ESP32.
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

/// `[]` Efuse write disable mask
pub const WR_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 16);
/// `[WR_DIS.EFUSE_RD_DISABLE]` wr_dis of RD_DIS
pub const WR_DIS_RD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 0, 1);
/// `[]` wr_dis of WR_DIS
pub const WR_DIS_WR_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 1, 1);
/// `[]` wr_dis of FLASH_CRYPT_CNT
pub const WR_DIS_FLASH_CRYPT_CNT: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[]` wr_dis of UART_DOWNLOAD_DIS
pub const WR_DIS_UART_DOWNLOAD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 2, 1);
/// `[WR_DIS.MAC_FACTORY]` wr_dis of MAC
pub const WR_DIS_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[WR_DIS.MAC_FACTORY_CRC]` wr_dis of MAC_CRC
pub const WR_DIS_MAC_CRC: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[WR_DIS.CHIP_VER_DIS_APP_CPU]` wr_dis of DISABLE_APP_CPU
pub const WR_DIS_DISABLE_APP_CPU: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[WR_DIS.CHIP_VER_DIS_BT]` wr_dis of DISABLE_BT
pub const WR_DIS_DISABLE_BT: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[WR_DIS.CHIP_VER_DIS_CACHE]` wr_dis of DIS_CACHE
pub const WR_DIS_DIS_CACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[]` wr_dis of VOL_LEVEL_HP_INV
pub const WR_DIS_VOL_LEVEL_HP_INV: EfuseField = EfuseField::new(EfuseBlock::Block0, 3, 1);
/// `[WR_DIS.CK8M_FREQ]` wr_dis of CLK8M_FREQ
pub const WR_DIS_CLK8M_FREQ: EfuseField = EfuseField::new(EfuseBlock::Block0, 4, 1);
/// `[]` wr_dis of ADC_VREF
pub const WR_DIS_ADC_VREF: EfuseField = EfuseField::new(EfuseBlock::Block0, 4, 1);
/// `[]` wr_dis of XPD_SDIO_REG
pub const WR_DIS_XPD_SDIO_REG: EfuseField = EfuseField::new(EfuseBlock::Block0, 5, 1);
/// `[WR_DIS.SDIO_TIEH]` wr_dis of XPD_SDIO_TIEH
pub const WR_DIS_XPD_SDIO_TIEH: EfuseField = EfuseField::new(EfuseBlock::Block0, 5, 1);
/// `[WR_DIS.SDIO_FORCE]` wr_dis of XPD_SDIO_FORCE
pub const WR_DIS_XPD_SDIO_FORCE: EfuseField = EfuseField::new(EfuseBlock::Block0, 5, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_CLK
pub const WR_DIS_SPI_PAD_CONFIG_CLK: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_Q
pub const WR_DIS_SPI_PAD_CONFIG_Q: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_D
pub const WR_DIS_SPI_PAD_CONFIG_D: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[]` wr_dis of SPI_PAD_CONFIG_CS0
pub const WR_DIS_SPI_PAD_CONFIG_CS0: EfuseField = EfuseField::new(EfuseBlock::Block0, 6, 1);
/// `[WR_DIS.ENCRYPT_FLASH_KEY WR_DIS.BLK1]` wr_dis of BLOCK1
pub const WR_DIS_BLOCK1: EfuseField = EfuseField::new(EfuseBlock::Block0, 7, 1);
/// `[WR_DIS.SECURE_BOOT_KEY WR_DIS.BLK2]` wr_dis of BLOCK2
pub const WR_DIS_BLOCK2: EfuseField = EfuseField::new(EfuseBlock::Block0, 8, 1);
/// `[WR_DIS.BLK3]` wr_dis of BLOCK3
pub const WR_DIS_BLOCK3: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[WR_DIS.MAC_CUSTOM_CRC]` wr_dis of CUSTOM_MAC_CRC
pub const WR_DIS_CUSTOM_MAC_CRC: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[WR_DIS.MAC_CUSTOM]` wr_dis of CUSTOM_MAC
pub const WR_DIS_CUSTOM_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[]` wr_dis of ADC1_TP_LOW
pub const WR_DIS_ADC1_TP_LOW: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[]` wr_dis of ADC1_TP_HIGH
pub const WR_DIS_ADC1_TP_HIGH: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[]` wr_dis of ADC2_TP_LOW
pub const WR_DIS_ADC2_TP_LOW: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[]` wr_dis of ADC2_TP_HIGH
pub const WR_DIS_ADC2_TP_HIGH: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[]` wr_dis of SECURE_VERSION
pub const WR_DIS_SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[WR_DIS.MAC_CUSTOM_VER]` wr_dis of MAC_VERSION
pub const WR_DIS_MAC_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 9, 1);
/// `[]` wr_dis of BLK3_PART_RESERVE
pub const WR_DIS_BLK3_PART_RESERVE: EfuseField = EfuseField::new(EfuseBlock::Block0, 10, 1);
/// `[WR_DIS.ENCRYPT_CONFIG]` wr_dis of FLASH_CRYPT_CONFIG
pub const WR_DIS_FLASH_CRYPT_CONFIG: EfuseField = EfuseField::new(EfuseBlock::Block0, 10, 1);
/// `[]` wr_dis of CODING_SCHEME
pub const WR_DIS_CODING_SCHEME: EfuseField = EfuseField::new(EfuseBlock::Block0, 10, 1);
/// `[]` wr_dis of KEY_STATUS
pub const WR_DIS_KEY_STATUS: EfuseField = EfuseField::new(EfuseBlock::Block0, 10, 1);
/// `[]` wr_dis of ABS_DONE_0
pub const WR_DIS_ABS_DONE_0: EfuseField = EfuseField::new(EfuseBlock::Block0, 12, 1);
/// `[]` wr_dis of ABS_DONE_1
pub const WR_DIS_ABS_DONE_1: EfuseField = EfuseField::new(EfuseBlock::Block0, 13, 1);
/// `[WR_DIS.DISABLE_JTAG]` wr_dis of JTAG_DISABLE
pub const WR_DIS_JTAG_DISABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 14, 1);
/// `[]` wr_dis of CONSOLE_DEBUG_DISABLE
pub const WR_DIS_CONSOLE_DEBUG_DISABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 15, 1);
/// `[]` wr_dis of DISABLE_DL_ENCRYPT
pub const WR_DIS_DISABLE_DL_ENCRYPT: EfuseField = EfuseField::new(EfuseBlock::Block0, 15, 1);
/// `[]` wr_dis of DISABLE_DL_DECRYPT
pub const WR_DIS_DISABLE_DL_DECRYPT: EfuseField = EfuseField::new(EfuseBlock::Block0, 15, 1);
/// `[]` wr_dis of DISABLE_DL_CACHE
pub const WR_DIS_DISABLE_DL_CACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 15, 1);
/// `[]` Disable reading from BlOCK1-3
pub const RD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 16, 4);
/// `[RD_DIS.ENCRYPT_FLASH_KEY RD_DIS.BLK1]` rd_dis of BLOCK1
pub const RD_DIS_BLOCK1: EfuseField = EfuseField::new(EfuseBlock::Block0, 16, 1);
/// `[RD_DIS.SECURE_BOOT_KEY RD_DIS.BLK2]` rd_dis of BLOCK2
pub const RD_DIS_BLOCK2: EfuseField = EfuseField::new(EfuseBlock::Block0, 17, 1);
/// `[RD_DIS.BLK3]` rd_dis of BLOCK3
pub const RD_DIS_BLOCK3: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[RD_DIS.MAC_CUSTOM_CRC]` rd_dis of CUSTOM_MAC_CRC
pub const RD_DIS_CUSTOM_MAC_CRC: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[RD_DIS.MAC_CUSTOM]` rd_dis of CUSTOM_MAC
pub const RD_DIS_CUSTOM_MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` rd_dis of ADC1_TP_LOW
pub const RD_DIS_ADC1_TP_LOW: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` rd_dis of ADC1_TP_HIGH
pub const RD_DIS_ADC1_TP_HIGH: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` rd_dis of ADC2_TP_LOW
pub const RD_DIS_ADC2_TP_LOW: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` rd_dis of ADC2_TP_HIGH
pub const RD_DIS_ADC2_TP_HIGH: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` rd_dis of SECURE_VERSION
pub const RD_DIS_SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[RD_DIS.MAC_CUSTOM_VER]` rd_dis of MAC_VERSION
pub const RD_DIS_MAC_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block0, 18, 1);
/// `[]` rd_dis of BLK3_PART_RESERVE
pub const RD_DIS_BLK3_PART_RESERVE: EfuseField = EfuseField::new(EfuseBlock::Block0, 19, 1);
/// `[RD_DIS.ENCRYPT_CONFIG]` rd_dis of FLASH_CRYPT_CONFIG
pub const RD_DIS_FLASH_CRYPT_CONFIG: EfuseField = EfuseField::new(EfuseBlock::Block0, 19, 1);
/// `[]` rd_dis of CODING_SCHEME
pub const RD_DIS_CODING_SCHEME: EfuseField = EfuseField::new(EfuseBlock::Block0, 19, 1);
/// `[]` rd_dis of KEY_STATUS
pub const RD_DIS_KEY_STATUS: EfuseField = EfuseField::new(EfuseBlock::Block0, 19, 1);
/// `[]` Flash encryption is enabled if this field has an odd number of bits set
pub const FLASH_CRYPT_CNT: EfuseField = EfuseField::new(EfuseBlock::Block0, 20, 7);
/// `[]` Disable UART download mode. Valid for ESP32 V3 and newer; only
pub const UART_DOWNLOAD_DIS: EfuseField = EfuseField::new(EfuseBlock::Block0, 27, 1);
/// `[MAC_FACTORY]` MAC address
pub const MAC: EfuseField = EfuseField::new(EfuseBlock::Block0, 32, 48);
/// `[MAC_FACTORY_CRC]` CRC8 for MAC address
pub const MAC_CRC: EfuseField = EfuseField::new(EfuseBlock::Block0, 80, 8);
/// `[CHIP_VER_DIS_APP_CPU]` Disables APP CPU
pub const DISABLE_APP_CPU: EfuseField = EfuseField::new(EfuseBlock::Block0, 96, 1);
/// `[CHIP_VER_DIS_BT]` Disables Bluetooth
pub const DISABLE_BT: EfuseField = EfuseField::new(EfuseBlock::Block0, 97, 1);
/// `[CHIP_VER_PKG_4BIT]` Chip package identifier
pub const CHIP_PACKAGE_4BIT: EfuseField = EfuseField::new(EfuseBlock::Block0, 98, 1);
/// `[CHIP_VER_DIS_CACHE]` Disables cache
pub const DIS_CACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 99, 1);
/// `[]` read for SPI_pad_config_hd
pub const SPI_PAD_CONFIG_HD: EfuseField = EfuseField::new(EfuseBlock::Block0, 100, 5);
/// `[CHIP_VER_PKG]` Chip package identifier
pub const CHIP_PACKAGE: EfuseField = EfuseField::new(EfuseBlock::Block0, 105, 3);
/// `[]` If set alongside EFUSE_RD_CHIP_CPU_FREQ_RATED; the ESP32's max CPU
/// frequency is rated for 160MHz. 240MHz otherwise
pub const CHIP_CPU_FREQ_LOW: EfuseField = EfuseField::new(EfuseBlock::Block0, 108, 1);
/// `[]` If set; the ESP32's maximum CPU frequency has been rated
pub const CHIP_CPU_FREQ_RATED: EfuseField = EfuseField::new(EfuseBlock::Block0, 109, 1);
/// `[]` BLOCK3 partially served for ADC calibration data
pub const BLK3_PART_RESERVE: EfuseField = EfuseField::new(EfuseBlock::Block0, 110, 1);
/// `[]` bit is set to 1 for rev1 silicon
pub const CHIP_VER_REV1: EfuseField = EfuseField::new(EfuseBlock::Block0, 111, 1);
/// `[CK8M_FREQ]` 8MHz clock freq override
pub const CLK8M_FREQ: EfuseField = EfuseField::new(EfuseBlock::Block0, 128, 8);
/// `[]` True ADC reference voltage
pub const ADC_VREF: EfuseField = EfuseField::new(EfuseBlock::Block0, 136, 5);
/// `[]` read for XPD_SDIO_REG
pub const XPD_SDIO_REG: EfuseField = EfuseField::new(EfuseBlock::Block0, 142, 1);
/// `[SDIO_TIEH]` If XPD_SDIO_FORCE & XPD_SDIO_REG {1: "3.3V"; 0: "1.8V"}
pub const XPD_SDIO_TIEH: EfuseField = EfuseField::new(EfuseBlock::Block0, 143, 1);
/// `[SDIO_FORCE]` Ignore MTDI pin (GPIO12) for VDD_SDIO on reset
pub const XPD_SDIO_FORCE: EfuseField = EfuseField::new(EfuseBlock::Block0, 144, 1);
/// `[]` Override SD_CLK pad (GPIO6/SPICLK)
pub const SPI_PAD_CONFIG_CLK: EfuseField = EfuseField::new(EfuseBlock::Block0, 160, 5);
/// `[]` Override SD_DATA_0 pad (GPIO7/SPIQ)
pub const SPI_PAD_CONFIG_Q: EfuseField = EfuseField::new(EfuseBlock::Block0, 165, 5);
/// `[]` Override SD_DATA_1 pad (GPIO8/SPID)
pub const SPI_PAD_CONFIG_D: EfuseField = EfuseField::new(EfuseBlock::Block0, 170, 5);
/// `[]` Override SD_CMD pad (GPIO11/SPICS0)
pub const SPI_PAD_CONFIG_CS0: EfuseField = EfuseField::new(EfuseBlock::Block0, 175, 5);
/// `[]`
pub const CHIP_VER_REV2: EfuseField = EfuseField::new(EfuseBlock::Block0, 180, 1);
/// `[]` This field stores the voltage level for CPU to run at 240 MHz; or for
/// flash/PSRAM to run at 80 MHz.0x0: level 7; 0x1: level 6; 0x2: level 5; 0x3:
/// level 4. (RO)
pub const VOL_LEVEL_HP_INV: EfuseField = EfuseField::new(EfuseBlock::Block0, 182, 2);
/// `[]`
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(EfuseBlock::Block0, 184, 2);
/// `[ENCRYPT_CONFIG]` Flash encryption config (key tweak bits)
pub const FLASH_CRYPT_CONFIG: EfuseField = EfuseField::new(EfuseBlock::Block0, 188, 4);
/// `[]` Efuse variable block length scheme {0: "NONE (BLK1-3 len=256 bits)"; 1:
/// "3/4 (BLK1-3 len=192 bits)"; 2: "REPEAT (BLK1-3 len=128 bits) not
/// supported"; 3: "NONE (BLK1-3 len=256 bits)"}
pub const CODING_SCHEME: EfuseField = EfuseField::new(EfuseBlock::Block0, 192, 2);
/// `[]` Disable ROM BASIC interpreter fallback
pub const CONSOLE_DEBUG_DISABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 194, 1);
/// `[]`
pub const DISABLE_SDIO_HOST: EfuseField = EfuseField::new(EfuseBlock::Block0, 195, 1);
/// `[]` Secure boot V1 is enabled for bootloader image
pub const ABS_DONE_0: EfuseField = EfuseField::new(EfuseBlock::Block0, 196, 1);
/// `[]` Secure boot V2 is enabled for bootloader image
pub const ABS_DONE_1: EfuseField = EfuseField::new(EfuseBlock::Block0, 197, 1);
/// `[DISABLE_JTAG]` Disable JTAG
pub const JTAG_DISABLE: EfuseField = EfuseField::new(EfuseBlock::Block0, 198, 1);
/// `[]` Disable flash encryption in UART bootloader
pub const DISABLE_DL_ENCRYPT: EfuseField = EfuseField::new(EfuseBlock::Block0, 199, 1);
/// `[]` Disable flash decryption in UART bootloader
pub const DISABLE_DL_DECRYPT: EfuseField = EfuseField::new(EfuseBlock::Block0, 200, 1);
/// `[]` Disable flash cache in UART bootloader
pub const DISABLE_DL_CACHE: EfuseField = EfuseField::new(EfuseBlock::Block0, 201, 1);
/// `[]` Usage of efuse block 3 (reserved)
pub const KEY_STATUS: EfuseField = EfuseField::new(EfuseBlock::Block0, 202, 1);
/// `[MAC_CUSTOM_CRC]` CRC8 for custom MAC address
pub const CUSTOM_MAC_CRC: EfuseField = EfuseField::new(EfuseBlock::Block3, 0, 8);
/// `[MAC_CUSTOM]` Custom MAC address
pub const MAC_CUSTOM: EfuseField = EfuseField::new(EfuseBlock::Block3, 8, 48);
/// `[]` ADC1 Two Point calibration low point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC1_TP_LOW: EfuseField = EfuseField::new(EfuseBlock::Block3, 96, 7);
/// `[]` ADC1 Two Point calibration high point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC1_TP_HIGH: EfuseField = EfuseField::new(EfuseBlock::Block3, 103, 9);
/// `[]` ADC2 Two Point calibration low point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC2_TP_LOW: EfuseField = EfuseField::new(EfuseBlock::Block3, 112, 7);
/// `[]` ADC2 Two Point calibration high point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC2_TP_HIGH: EfuseField = EfuseField::new(EfuseBlock::Block3, 119, 9);
/// `[]` Secure version for anti-rollback
pub const SECURE_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block3, 128, 32);
/// `[MAC_CUSTOM_VER]` Version of the MAC field {1: "Custom MAC in BLOCK3"}
pub const MAC_VERSION: EfuseField = EfuseField::new(EfuseBlock::Block3, 184, 8);

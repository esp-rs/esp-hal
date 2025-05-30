//! This file was automatically generated, please do not edit it manually!
//!
//! Generated: 2025-05-30 12:24
//! Version:   369d2d860d34e777c0f7d545a7dfc3c4

#![allow(clippy::empty_docs)]

use super::EfuseField;

/// Efuse write disable mask
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 16);
/// Disable reading from BlOCK1-3
pub const RD_DIS: EfuseField = EfuseField::new(0, 0, 16, 4);
/// Flash encryption is enabled if this field has an odd number of bits set
pub const FLASH_CRYPT_CNT: EfuseField = EfuseField::new(0, 0, 20, 7);
/// Disable UART download mode. Valid for ESP32 V3 and newer; only
pub const UART_DOWNLOAD_DIS: EfuseField = EfuseField::new(0, 0, 27, 1);
/// reserved
pub const RESERVED_0_28: EfuseField = EfuseField::new(0, 0, 28, 4);
/// MAC address
pub const MAC0: EfuseField = EfuseField::new(0, 1, 32, 32);
/// MAC address
pub const MAC1: EfuseField = EfuseField::new(0, 2, 64, 16);
/// CRC8 for MAC address
pub const MAC_CRC: EfuseField = EfuseField::new(0, 2, 80, 8);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_88: EfuseField = EfuseField::new(0, 2, 88, 8);
/// Disables APP CPU
pub const DISABLE_APP_CPU: EfuseField = EfuseField::new(0, 3, 96, 1);
/// Disables Bluetooth
pub const DISABLE_BT: EfuseField = EfuseField::new(0, 3, 97, 1);
/// Chip package identifier #4bit
pub const CHIP_PACKAGE_4BIT: EfuseField = EfuseField::new(0, 3, 98, 1);
/// Disables cache
pub const DIS_CACHE: EfuseField = EfuseField::new(0, 3, 99, 1);
/// read for SPI_pad_config_hd
pub const SPI_PAD_CONFIG_HD: EfuseField = EfuseField::new(0, 3, 100, 5);
/// Chip package identifier
pub const CHIP_PACKAGE: EfuseField = EfuseField::new(0, 3, 105, 3);
/// If set alongside EFUSE_RD_CHIP_CPU_FREQ_RATED; the ESP32's max CPU frequency
/// is rated for 160MHz. 240MHz otherwise
pub const CHIP_CPU_FREQ_LOW: EfuseField = EfuseField::new(0, 3, 108, 1);
/// If set; the ESP32's maximum CPU frequency has been rated
pub const CHIP_CPU_FREQ_RATED: EfuseField = EfuseField::new(0, 3, 109, 1);
/// BLOCK3 partially served for ADC calibration data
pub const BLK3_PART_RESERVE: EfuseField = EfuseField::new(0, 3, 110, 1);
/// bit is set to 1 for rev1 silicon
pub const CHIP_VER_REV1: EfuseField = EfuseField::new(0, 3, 111, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_112: EfuseField = EfuseField::new(0, 3, 112, 16);
/// 8MHz clock freq override
pub const CLK8M_FREQ: EfuseField = EfuseField::new(0, 4, 128, 8);
/// True ADC reference voltage
pub const ADC_VREF: EfuseField = EfuseField::new(0, 4, 136, 5);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_141: EfuseField = EfuseField::new(0, 4, 141, 1);
/// read for XPD_SDIO_REG
pub const XPD_SDIO_REG: EfuseField = EfuseField::new(0, 4, 142, 1);
/// If XPD_SDIO_FORCE & XPD_SDIO_REG
pub const XPD_SDIO_TIEH: EfuseField = EfuseField::new(0, 4, 143, 1);
/// Ignore MTDI pin (GPIO12) for VDD_SDIO on reset
pub const XPD_SDIO_FORCE: EfuseField = EfuseField::new(0, 4, 144, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_145: EfuseField = EfuseField::new(0, 4, 145, 15);
/// Override SD_CLK pad (GPIO6/SPICLK)
pub const SPI_PAD_CONFIG_CLK: EfuseField = EfuseField::new(0, 5, 160, 5);
/// Override SD_DATA_0 pad (GPIO7/SPIQ)
pub const SPI_PAD_CONFIG_Q: EfuseField = EfuseField::new(0, 5, 165, 5);
/// Override SD_DATA_1 pad (GPIO8/SPID)
pub const SPI_PAD_CONFIG_D: EfuseField = EfuseField::new(0, 5, 170, 5);
/// Override SD_CMD pad (GPIO11/SPICS0)
pub const SPI_PAD_CONFIG_CS0: EfuseField = EfuseField::new(0, 5, 175, 5);
///
pub const CHIP_VER_REV2: EfuseField = EfuseField::new(0, 5, 180, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_181: EfuseField = EfuseField::new(0, 5, 181, 1);
/// This field stores the voltage level for CPU to run at 240 MHz; or for
/// flash/PSRAM to run at 80 MHz.0x0: level 7; 0x1: level 6; 0x2: level 5; 0x3:
/// level 4. (RO)
pub const VOL_LEVEL_HP_INV: EfuseField = EfuseField::new(0, 5, 182, 2);
///
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(0, 5, 184, 2);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_186: EfuseField = EfuseField::new(0, 5, 186, 2);
/// Flash encryption config (key tweak bits)
pub const FLASH_CRYPT_CONFIG: EfuseField = EfuseField::new(0, 5, 188, 4);
/// Efuse variable block length scheme
pub const CODING_SCHEME: EfuseField = EfuseField::new(0, 6, 192, 2);
/// Disable ROM BASIC interpreter fallback
pub const CONSOLE_DEBUG_DISABLE: EfuseField = EfuseField::new(0, 6, 194, 1);
///
pub const DISABLE_SDIO_HOST: EfuseField = EfuseField::new(0, 6, 195, 1);
/// Secure boot V1 is enabled for bootloader image
pub const ABS_DONE_0: EfuseField = EfuseField::new(0, 6, 196, 1);
/// Secure boot V2 is enabled for bootloader image
pub const ABS_DONE_1: EfuseField = EfuseField::new(0, 6, 197, 1);
/// Disable JTAG
pub const JTAG_DISABLE: EfuseField = EfuseField::new(0, 6, 198, 1);
/// Disable flash encryption in UART bootloader
pub const DISABLE_DL_ENCRYPT: EfuseField = EfuseField::new(0, 6, 199, 1);
/// Disable flash decryption in UART bootloader
pub const DISABLE_DL_DECRYPT: EfuseField = EfuseField::new(0, 6, 200, 1);
/// Disable flash cache in UART bootloader
pub const DISABLE_DL_CACHE: EfuseField = EfuseField::new(0, 6, 201, 1);
/// Usage of efuse block 3 (reserved)
pub const KEY_STATUS: EfuseField = EfuseField::new(0, 6, 202, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_203: EfuseField = EfuseField::new(0, 6, 203, 21);
/// Flash encryption key
pub const BLOCK1: EfuseField = EfuseField::new(1, 0, 0, 256);
/// Security boot key
pub const BLOCK2: EfuseField = EfuseField::new(2, 0, 0, 256);
/// CRC8 for custom MAC address
pub const CUSTOM_MAC_CRC: EfuseField = EfuseField::new(3, 0, 0, 8);
/// Custom MAC address
pub const CUSTOM_MAC: EfuseField = EfuseField::new(3, 0, 8, 48);
/// reserved
pub const RESERVED_3_56: EfuseField = EfuseField::new(3, 1, 56, 8);
/// read for BLOCK3
pub const BLK3_RESERVED_2: EfuseField = EfuseField::new(3, 2, 64, 32);
/// ADC1 Two Point calibration low point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC1_TP_LOW: EfuseField = EfuseField::new(3, 3, 96, 7);
/// ADC1 Two Point calibration high point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC1_TP_HIGH: EfuseField = EfuseField::new(3, 3, 103, 9);
/// ADC2 Two Point calibration low point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC2_TP_LOW: EfuseField = EfuseField::new(3, 3, 112, 7);
/// ADC2 Two Point calibration high point. Only valid if
/// EFUSE_RD_BLK3_PART_RESERVE
pub const ADC2_TP_HIGH: EfuseField = EfuseField::new(3, 3, 119, 9);
/// Secure version for anti-rollback
pub const SECURE_VERSION: EfuseField = EfuseField::new(3, 4, 128, 32);
/// reserved
pub const RESERVED_3_160: EfuseField = EfuseField::new(3, 5, 160, 24);
/// Version of the MAC field
pub const MAC_VERSION: EfuseField = EfuseField::new(3, 5, 184, 8);
/// read for BLOCK3
pub const BLK3_RESERVED_6: EfuseField = EfuseField::new(3, 6, 192, 32);
/// read for BLOCK3
pub const BLK3_RESERVED_7: EfuseField = EfuseField::new(3, 7, 224, 32);

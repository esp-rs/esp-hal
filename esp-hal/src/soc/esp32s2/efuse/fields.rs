//! This file was automatically generated, please do not edit it manually!
//!
//! Generated: 2025-04-22 11:33
//! Version:   888a61f6f500d9c7ee0aa32016b0bee7

#![allow(clippy::empty_docs)]

use super::EfuseField;

/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// Reserved
pub const DIS_RTC_RAM_BOOT: EfuseField = EfuseField::new(0, 1, 39, 1);
/// Set this bit to disable Icache
pub const DIS_ICACHE: EfuseField = EfuseField::new(0, 1, 40, 1);
/// Set this bit to disable Dcache
pub const DIS_DCACHE: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Disables Icache when SoC is in Download mode
pub const DIS_DOWNLOAD_ICACHE: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Disables Dcache when SoC is in Download mode
pub const DIS_DOWNLOAD_DCACHE: EfuseField = EfuseField::new(0, 1, 43, 1);
/// Set this bit to disable the function that forces chip into download mode
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Set this bit to disable USB OTG function
pub const DIS_USB: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Set this bit to disable the TWAI Controller function
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Disables capability to Remap RAM to ROM address space
pub const DIS_BOOT_REMAP: EfuseField = EfuseField::new(0, 1, 47, 1);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED5: EfuseField = EfuseField::new(0, 1, 48, 1);
/// Software disables JTAG. When software disabled; JTAG can be activated
/// temporarily by HMAC peripheral
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 49, 1);
/// Hardware disables JTAG permanently
pub const HARD_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 50, 1);
/// Disables flash encryption when in download boot modes
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Controls single-end input threshold vrefh; 1.76 V to 2 V with step of 80 mV;
/// stored in eFuse
pub const USB_DREFH: EfuseField = EfuseField::new(0, 1, 52, 2);
/// Controls single-end input threshold vrefl; 0.8 V to 1.04 V with step of 80
/// mV; stored in eFuse
pub const USB_DREFL: EfuseField = EfuseField::new(0, 1, 54, 2);
/// Set this bit to exchange USB D+ and D- pins
pub const USB_EXCHG_PINS: EfuseField = EfuseField::new(0, 1, 56, 1);
/// Set this bit to enable external USB PHY
pub const USB_EXT_PHY_ENABLE: EfuseField = EfuseField::new(0, 1, 57, 1);
/// If set; forces USB BVALID to 1
pub const USB_FORCE_NOPERSIST: EfuseField = EfuseField::new(0, 1, 58, 1);
/// BLOCK0 efuse version
pub const BLOCK0_VERSION: EfuseField = EfuseField::new(0, 1, 59, 2);
/// SPI regulator switches current limit mode
pub const VDD_SPI_MODECURLIM: EfuseField = EfuseField::new(0, 1, 61, 1);
/// SPI regulator high voltage reference
pub const VDD_SPI_DREFH: EfuseField = EfuseField::new(0, 1, 62, 2);
/// SPI regulator medium voltage reference
pub const VDD_SPI_DREFM: EfuseField = EfuseField::new(0, 2, 64, 2);
/// SPI regulator low voltage reference
pub const VDD_SPI_DREFL: EfuseField = EfuseField::new(0, 2, 66, 2);
/// If VDD_SPI_FORCE is 1; this value determines if the VDD_SPI regulator is
/// powered on
pub const VDD_SPI_XPD: EfuseField = EfuseField::new(0, 2, 68, 1);
/// If VDD_SPI_FORCE is 1; determines VDD_SPI voltage
pub const VDD_SPI_TIEH: EfuseField = EfuseField::new(0, 2, 69, 1);
/// Set this bit to use XPD_VDD_PSI_REG and VDD_SPI_TIEH to configure VDD_SPI
/// LDO
pub const VDD_SPI_FORCE: EfuseField = EfuseField::new(0, 2, 70, 1);
/// Set SPI regulator to 0 to configure init[1:0]=0
pub const VDD_SPI_EN_INIT: EfuseField = EfuseField::new(0, 2, 71, 1);
/// Set SPI regulator to 1 to enable output current limit
pub const VDD_SPI_ENCURLIM: EfuseField = EfuseField::new(0, 2, 72, 1);
/// Tunes the current limit threshold of SPI regulator when tieh=0; about 800
/// mA/(8+d)
pub const VDD_SPI_DCURLIM: EfuseField = EfuseField::new(0, 2, 73, 3);
/// Adds resistor from LDO output to ground
pub const VDD_SPI_INIT: EfuseField = EfuseField::new(0, 2, 76, 2);
/// Prevents SPI regulator from overshoot
pub const VDD_SPI_DCAP: EfuseField = EfuseField::new(0, 2, 78, 2);
/// RTC watchdog timeout threshold; in unit of slow clock cycle
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 2, 80, 2);
/// Enables flash encryption when 1 or 3 bits are set and disabled otherwise
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 2, 82, 3);
/// Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 2, 85, 1);
/// Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 2, 86, 1);
/// Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 2, 87, 1);
/// Purpose of KEY0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 2, 88, 4);
/// Purpose of KEY1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 2, 92, 4);
/// Purpose of KEY2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 3, 96, 4);
/// Purpose of KEY3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 3, 100, 4);
/// Purpose of KEY4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 3, 104, 4);
/// Purpose of KEY5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 3, 108, 4);
/// Purpose of KEY6
pub const KEY_PURPOSE_6: EfuseField = EfuseField::new(0, 3, 112, 4);
/// Set this bit to enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 116, 1);
/// Set this bit to enable aggressive secure boot key revocation mode
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 117, 1);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED1: EfuseField = EfuseField::new(0, 3, 118, 6);
/// Configures flash startup delay after SoC power-up; in unit of (ms/2). When
/// the value is 15; delay is 7.5 ms
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 124, 4);
/// Set this bit to disable all download boot modes
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Set this bit to disable Legacy SPI boot mode
pub const DIS_LEGACY_SPI_BOOT: EfuseField = EfuseField::new(0, 4, 129, 1);
/// Selects the default UART for printing boot messages
pub const UART_PRINT_CHANNEL: EfuseField = EfuseField::new(0, 4, 130, 1);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED3: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Set this bit to disable use of USB OTG in UART download boot mode
pub const DIS_USB_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Set this bit to enable secure UART download mode (read/write flash only)
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Set the default UART boot message output mode
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 134, 2);
/// Set default power supply for GPIO33-GPIO37; set when SPI flash is
/// initialized
pub const PIN_POWER_SELECTION: EfuseField = EfuseField::new(0, 4, 136, 1);
/// SPI flash type
pub const FLASH_TYPE: EfuseField = EfuseField::new(0, 4, 137, 1);
/// If set; forces ROM code to send an SPI flash resume command during SPI boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 138, 1);
/// Secure version (used by ESP-IDF anti-rollback feature)
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 139, 16);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED2: EfuseField = EfuseField::new(0, 4, 155, 5);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(0, 5, 160, 1);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(0, 5, 161, 1);
/// reserved
pub const RESERVED_0_162: EfuseField = EfuseField::new(0, 5, 162, 30);
/// MAC address
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// MAC address
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// SPI_PAD_configure CLK
pub const SPI_PAD_CONFIG_CLK: EfuseField = EfuseField::new(1, 1, 48, 6);
/// SPI_PAD_configure Q(D1)
pub const SPI_PAD_CONFIG_Q: EfuseField = EfuseField::new(1, 1, 54, 6);
/// SPI_PAD_configure D(D0)
pub const SPI_PAD_CONFIG_D: EfuseField = EfuseField::new(1, 1, 60, 6);
/// SPI_PAD_configure CS
pub const SPI_PAD_CONFIG_CS: EfuseField = EfuseField::new(1, 2, 66, 6);
/// SPI_PAD_configure HD(D3)
pub const SPI_PAD_CONFIG_HD: EfuseField = EfuseField::new(1, 2, 72, 6);
/// SPI_PAD_configure WP(D2)
pub const SPI_PAD_CONFIG_WP: EfuseField = EfuseField::new(1, 2, 78, 6);
/// SPI_PAD_configure DQS
pub const SPI_PAD_CONFIG_DQS: EfuseField = EfuseField::new(1, 2, 84, 6);
/// SPI_PAD_configure D4
pub const SPI_PAD_CONFIG_D4: EfuseField = EfuseField::new(1, 2, 90, 6);
/// SPI_PAD_configure D5
pub const SPI_PAD_CONFIG_D5: EfuseField = EfuseField::new(1, 3, 96, 6);
/// SPI_PAD_configure D6
pub const SPI_PAD_CONFIG_D6: EfuseField = EfuseField::new(1, 3, 102, 6);
/// SPI_PAD_configure D7
pub const SPI_PAD_CONFIG_D7: EfuseField = EfuseField::new(1, 3, 108, 6);
/// WAFER_VERSION_MAJOR
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 114, 2);
/// WAFER_VERSION_MINOR most significant bit
pub const WAFER_VERSION_MINOR_HI: EfuseField = EfuseField::new(1, 3, 116, 1);
/// Flash version
pub const FLASH_VERSION: EfuseField = EfuseField::new(1, 3, 117, 4);
/// BLK_VERSION_MAJOR
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 121, 2);
/// reserved
pub const RESERVED_1_123: EfuseField = EfuseField::new(1, 3, 123, 1);
/// PSRAM version
pub const PSRAM_VERSION: EfuseField = EfuseField::new(1, 3, 124, 4);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 4, 128, 4);
/// WAFER_VERSION_MINOR least significant bits
pub const WAFER_VERSION_MINOR_LO: EfuseField = EfuseField::new(1, 4, 132, 3);
/// reserved
pub const RESERVED_1_135: EfuseField = EfuseField::new(1, 4, 135, 25);
/// Stores the second part of the zeroth part of system data
pub const SYS_DATA_PART0_2: EfuseField = EfuseField::new(1, 5, 160, 32);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// 4 bit of ADC calibration
pub const ADC_CALIB: EfuseField = EfuseField::new(2, 4, 128, 4);
/// BLK_VERSION_MINOR of BLOCK2
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(2, 4, 132, 3);
/// Temperature calibration data
pub const TEMP_CALIB: EfuseField = EfuseField::new(2, 4, 135, 9);
///
pub const RTCCALIB_V1IDX_A10H: EfuseField = EfuseField::new(2, 4, 144, 8);
///
pub const RTCCALIB_V1IDX_A11H: EfuseField = EfuseField::new(2, 4, 152, 8);
///
pub const RTCCALIB_V1IDX_A12H: EfuseField = EfuseField::new(2, 5, 160, 8);
///
pub const RTCCALIB_V1IDX_A13H: EfuseField = EfuseField::new(2, 5, 168, 8);
///
pub const RTCCALIB_V1IDX_A20H: EfuseField = EfuseField::new(2, 5, 176, 8);
///
pub const RTCCALIB_V1IDX_A21H: EfuseField = EfuseField::new(2, 5, 184, 8);
///
pub const RTCCALIB_V1IDX_A22H: EfuseField = EfuseField::new(2, 6, 192, 8);
///
pub const RTCCALIB_V1IDX_A23H: EfuseField = EfuseField::new(2, 6, 200, 8);
///
pub const RTCCALIB_V1IDX_A10L: EfuseField = EfuseField::new(2, 6, 208, 6);
///
pub const RTCCALIB_V1IDX_A11L: EfuseField = EfuseField::new(2, 6, 214, 6);
///
pub const RTCCALIB_V1IDX_A12L: EfuseField = EfuseField::new(2, 6, 220, 6);
///
pub const RTCCALIB_V1IDX_A13L: EfuseField = EfuseField::new(2, 7, 226, 6);
///
pub const RTCCALIB_V1IDX_A20L: EfuseField = EfuseField::new(2, 7, 232, 6);
///
pub const RTCCALIB_V1IDX_A21L: EfuseField = EfuseField::new(2, 7, 238, 6);
///
pub const RTCCALIB_V1IDX_A22L: EfuseField = EfuseField::new(2, 7, 244, 6);
///
pub const RTCCALIB_V1IDX_A23L: EfuseField = EfuseField::new(2, 7, 250, 6);
/// User data
pub const BLOCK_USR_DATA: EfuseField = EfuseField::new(3, 0, 0, 192);
/// reserved
pub const RESERVED_3_192: EfuseField = EfuseField::new(3, 6, 192, 8);
/// Custom MAC
pub const CUSTOM_MAC: EfuseField = EfuseField::new(3, 6, 200, 48);
/// reserved
pub const RESERVED_3_248: EfuseField = EfuseField::new(3, 7, 248, 8);
/// Key0 or user data
pub const BLOCK_KEY0: EfuseField = EfuseField::new(4, 0, 0, 256);
/// Key1 or user data
pub const BLOCK_KEY1: EfuseField = EfuseField::new(5, 0, 0, 256);
/// Key2 or user data
pub const BLOCK_KEY2: EfuseField = EfuseField::new(6, 0, 0, 256);
/// Key3 or user data
pub const BLOCK_KEY3: EfuseField = EfuseField::new(7, 0, 0, 256);
/// Key4 or user data
pub const BLOCK_KEY4: EfuseField = EfuseField::new(8, 0, 0, 256);
/// Key5 or user data
pub const BLOCK_KEY5: EfuseField = EfuseField::new(9, 0, 0, 256);
/// System data part 2 (reserved)
pub const BLOCK_SYS_DATA2: EfuseField = EfuseField::new(10, 0, 0, 256);

//! This file was automatically generated, please do not edit it manually!
//!
//! Generated: 2025-04-22 11:33
//! Version:   4622cf9245401eca0eb1df8122449a6d

#![allow(clippy::empty_docs)]

use super::EfuseField;

/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// Set this bit to disable boot from RTC RAM
pub const DIS_RTC_RAM_BOOT: EfuseField = EfuseField::new(0, 1, 39, 1);
/// Set this bit to disable Icache
pub const DIS_ICACHE: EfuseField = EfuseField::new(0, 1, 40, 1);
/// Set this bit to disable function of usb switch to jtag in module of usb
/// device
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Set this bit to disable Icache in download mode (boot_mode[3:0] is 0; 1; 2;
/// 3; 6; 7)
pub const DIS_DOWNLOAD_ICACHE: EfuseField = EfuseField::new(0, 1, 42, 1);
/// USB-Serial-JTAG
pub const DIS_USB_SERIAL_JTAG: EfuseField = EfuseField::new(0, 1, 43, 1);
/// Set this bit to disable the function that forces chip into download mode
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED6: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Set this bit to disable CAN function
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Set this bit to enable selection between usb_to_jtag and pad_to_jtag through
/// strapping gpio10 when both reg_dis_usb_jtag and reg_dis_pad_jtag are equal
/// to 0
pub const JTAG_SEL_ENABLE: EfuseField = EfuseField::new(0, 1, 47, 1);
/// Set these bits to disable JTAG in the soft way (odd number 1 means disable
/// ). JTAG can be enabled in HMAC module
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 48, 3);
/// Set this bit to disable JTAG in the hard way. JTAG is disabled permanently
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Set this bit to disable flash encryption when in download boot modes
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 52, 1);
/// Controls single-end input threshold vrefh; 1.76 V to 2 V with step of 80 mV;
/// stored in eFuse
pub const USB_DREFH: EfuseField = EfuseField::new(0, 1, 53, 2);
/// Controls single-end input threshold vrefl; 0.8 V to 1.04 V with step of 80
/// mV; stored in eFuse
pub const USB_DREFL: EfuseField = EfuseField::new(0, 1, 55, 2);
/// Set this bit to exchange USB D+ and D- pins
pub const USB_EXCHG_PINS: EfuseField = EfuseField::new(0, 1, 57, 1);
/// Set this bit to vdd spi pin function as gpio
pub const VDD_SPI_AS_GPIO: EfuseField = EfuseField::new(0, 1, 58, 1);
/// Enable btlc gpio
pub const BTLC_GPIO_ENABLE: EfuseField = EfuseField::new(0, 1, 59, 2);
/// Set this bit to enable power glitch function
pub const POWERGLITCH_EN: EfuseField = EfuseField::new(0, 1, 61, 1);
/// Sample delay configuration of power glitch
pub const POWER_GLITCH_DSENSE: EfuseField = EfuseField::new(0, 1, 62, 2);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED2: EfuseField = EfuseField::new(0, 2, 64, 16);
/// RTC watchdog timeout threshold; in unit of slow clock cycle
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 2, 80, 2);
/// Enables flash encryption when 1 or 3 bits are set and disables otherwise
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 2, 82, 3);
/// Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 2, 85, 1);
/// Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 2, 86, 1);
/// Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 2, 87, 1);
/// Purpose of Key0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 2, 88, 4);
/// Purpose of Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 2, 92, 4);
/// Purpose of Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 3, 96, 4);
/// Purpose of Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 3, 100, 4);
/// Purpose of Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 3, 104, 4);
/// Purpose of Key5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 3, 108, 4);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED3: EfuseField = EfuseField::new(0, 3, 112, 4);
/// Set this bit to enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 116, 1);
/// Set this bit to enable revoking aggressive secure boot
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 117, 1);
/// Reserved (used for four backups method)
pub const RPT4_RESERVED0: EfuseField = EfuseField::new(0, 3, 118, 6);
/// Configures flash waiting time after power-up; in unit of ms. If the value is
/// less than 15; the waiting time is the configurable value; Otherwise; the
/// waiting time is twice the configurable value
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 124, 4);
/// Set this bit to disable download mode (boot_mode[3:0] = 0; 1; 2; 3; 6; 7)
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Disable direct boot mode
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 4, 129, 1);
/// USB printing
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 4, 130, 1);
/// ECC mode in ROM
pub const FLASH_ECC_MODE: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Disable UART download mode through USB-Serial-JTAG
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Set this bit to enable secure UART download mode
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Set the default UARTboot message output mode
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 134, 2);
/// GPIO33-GPIO37 power supply selection in ROM code
pub const PIN_POWER_SELECTION: EfuseField = EfuseField::new(0, 4, 136, 1);
/// Maximum lines of SPI flash
pub const FLASH_TYPE: EfuseField = EfuseField::new(0, 4, 137, 1);
/// Set Flash page size
pub const FLASH_PAGE_SIZE: EfuseField = EfuseField::new(0, 4, 138, 2);
/// Set 1 to enable ECC for flash boot
pub const FLASH_ECC_EN: EfuseField = EfuseField::new(0, 4, 140, 1);
/// Set this bit to force ROM code to send a resume command during SPI boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 141, 1);
/// Secure version (used by ESP-IDF anti-rollback feature)
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 142, 16);
/// reserved
pub const RESERVED_0_158: EfuseField = EfuseField::new(0, 4, 158, 1);
/// Use BLOCK0 to check error record registers
pub const ERR_RST_ENABLE: EfuseField = EfuseField::new(0, 4, 159, 1);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(0, 5, 160, 1);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(0, 5, 161, 1);
/// reserved
pub const RESERVED_0_162: EfuseField = EfuseField::new(0, 5, 162, 22);
/// MAC address
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// MAC address
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// SPI PAD CLK
pub const SPI_PAD_CONFIG_CLK: EfuseField = EfuseField::new(1, 1, 48, 6);
/// SPI PAD Q(D1)
pub const SPI_PAD_CONFIG_Q: EfuseField = EfuseField::new(1, 1, 54, 6);
/// SPI PAD D(D0)
pub const SPI_PAD_CONFIG_D: EfuseField = EfuseField::new(1, 1, 60, 6);
/// SPI PAD CS
pub const SPI_PAD_CONFIG_CS: EfuseField = EfuseField::new(1, 2, 66, 6);
/// SPI PAD HD(D3)
pub const SPI_PAD_CONFIG_HD: EfuseField = EfuseField::new(1, 2, 72, 6);
/// SPI PAD WP(D2)
pub const SPI_PAD_CONFIG_WP: EfuseField = EfuseField::new(1, 2, 78, 6);
/// SPI PAD DQS
pub const SPI_PAD_CONFIG_DQS: EfuseField = EfuseField::new(1, 2, 84, 6);
/// SPI PAD D4
pub const SPI_PAD_CONFIG_D4: EfuseField = EfuseField::new(1, 2, 90, 6);
/// SPI PAD D5
pub const SPI_PAD_CONFIG_D5: EfuseField = EfuseField::new(1, 3, 96, 6);
/// SPI PAD D6
pub const SPI_PAD_CONFIG_D6: EfuseField = EfuseField::new(1, 3, 102, 6);
/// SPI PAD D7
pub const SPI_PAD_CONFIG_D7: EfuseField = EfuseField::new(1, 3, 108, 6);
/// WAFER_VERSION_MINOR least significant bits
pub const WAFER_VERSION_MINOR_LO: EfuseField = EfuseField::new(1, 3, 114, 3);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 3, 117, 3);
/// BLK_VERSION_MINOR
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(1, 3, 120, 3);
/// Flash capacity
pub const FLASH_CAP: EfuseField = EfuseField::new(1, 3, 123, 3);
/// Flash temperature
pub const FLASH_TEMP: EfuseField = EfuseField::new(1, 3, 126, 2);
/// Flash vendor
pub const FLASH_VENDOR: EfuseField = EfuseField::new(1, 4, 128, 3);
/// reserved
pub const RESERVED_1_131: EfuseField = EfuseField::new(1, 4, 131, 4);
/// BLOCK1 K_RTC_LDO
pub const K_RTC_LDO: EfuseField = EfuseField::new(1, 4, 135, 7);
/// BLOCK1 K_DIG_LDO
pub const K_DIG_LDO: EfuseField = EfuseField::new(1, 4, 142, 7);
/// BLOCK1 voltage of rtc dbias20
pub const V_RTC_DBIAS20: EfuseField = EfuseField::new(1, 4, 149, 8);
/// BLOCK1 voltage of digital dbias20
pub const V_DIG_DBIAS20: EfuseField = EfuseField::new(1, 4, 157, 8);
/// BLOCK1 digital dbias when hvt
pub const DIG_DBIAS_HVT: EfuseField = EfuseField::new(1, 5, 165, 5);
/// BLOCK1 pvt threshold when hvt
pub const THRES_HVT: EfuseField = EfuseField::new(1, 5, 170, 10);
/// reserved
pub const RESERVED_1_180: EfuseField = EfuseField::new(1, 5, 180, 3);
/// WAFER_VERSION_MINOR most significant bit
pub const WAFER_VERSION_MINOR_HI: EfuseField = EfuseField::new(1, 5, 183, 1);
/// WAFER_VERSION_MAJOR
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 5, 184, 2);
/// reserved
pub const RESERVED_1_186: EfuseField = EfuseField::new(1, 5, 186, 6);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(2, 4, 128, 2);
/// reserved
pub const RESERVED_2_130: EfuseField = EfuseField::new(2, 4, 130, 1);
/// Temperature calibration data
pub const TEMP_CALIB: EfuseField = EfuseField::new(2, 4, 131, 9);
/// ADC OCode
pub const OCODE: EfuseField = EfuseField::new(2, 4, 140, 8);
/// ADC1 init code at atten0
pub const ADC1_INIT_CODE_ATTEN0: EfuseField = EfuseField::new(2, 4, 148, 10);
/// ADC1 init code at atten1
pub const ADC1_INIT_CODE_ATTEN1: EfuseField = EfuseField::new(2, 4, 158, 10);
/// ADC1 init code at atten2
pub const ADC1_INIT_CODE_ATTEN2: EfuseField = EfuseField::new(2, 5, 168, 10);
/// ADC1 init code at atten3
pub const ADC1_INIT_CODE_ATTEN3: EfuseField = EfuseField::new(2, 5, 178, 10);
/// ADC1 calibration voltage at atten0
pub const ADC1_CAL_VOL_ATTEN0: EfuseField = EfuseField::new(2, 5, 188, 10);
/// ADC1 calibration voltage at atten1
pub const ADC1_CAL_VOL_ATTEN1: EfuseField = EfuseField::new(2, 6, 198, 10);
/// ADC1 calibration voltage at atten2
pub const ADC1_CAL_VOL_ATTEN2: EfuseField = EfuseField::new(2, 6, 208, 10);
/// ADC1 calibration voltage at atten3
pub const ADC1_CAL_VOL_ATTEN3: EfuseField = EfuseField::new(2, 6, 218, 10);
/// reserved
pub const RESERVED_2_228: EfuseField = EfuseField::new(2, 7, 228, 28);
/// User data
pub const BLOCK_USR_DATA: EfuseField = EfuseField::new(3, 0, 0, 192);
/// reserved
pub const RESERVED_3_192: EfuseField = EfuseField::new(3, 6, 192, 8);
/// Custom MAC address
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

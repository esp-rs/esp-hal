//! This file was automatically generated, please do not edit it manually!
//!
//! Generated: 2025-04-22 11:33
//! Version:   44563d2af4ebdba4db6c0a34a50c94f9

#![allow(clippy::empty_docs)]

use super::EfuseField;

/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// Reserved
pub const RPT4_RESERVED0_4: EfuseField = EfuseField::new(0, 1, 39, 1);
/// Represents whether icache is disabled or enabled. 1: disabled. 0: enabled
pub const DIS_ICACHE: EfuseField = EfuseField::new(0, 1, 40, 1);
/// Represents whether the function of usb switch to jtag is disabled or
/// enabled. 1: disabled. 0: enabled
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Represents whether power glitch function is enabled. 1: enabled. 0: disabled
pub const POWERGLITCH_EN: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Represents whether USB-Serial-JTAG is disabled or enabled. 1: disabled. 0:
/// enabled
pub const DIS_USB_SERIAL_JTAG: EfuseField = EfuseField::new(0, 1, 43, 1);
/// Represents whether the function that forces chip into download mode is
/// disabled or enabled. 1: disabled. 0: enabled
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Represents whether SPI0 controller during boot_mode_download is disabled or
/// enabled. 1: disabled. 0: enabled
pub const SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Represents whether TWAI function is disabled or enabled. 1: disabled. 0:
/// enabled
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Set this bit to enable selection between usb_to_jtag and pad_to_jtag through
/// strapping gpio25 when both EFUSE_DIS_PAD_JTAG and EFUSE_DIS_USB_JTAG are
/// equal to 0
pub const JTAG_SEL_ENABLE: EfuseField = EfuseField::new(0, 1, 47, 1);
/// Represents whether JTAG is disabled in soft way. Odd number: disabled. Even
/// number: enabled
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 48, 3);
/// Represents whether JTAG is disabled in the hard way(permanently). 1:
/// disabled. 0: enabled
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Represents whether flash encrypt function is disabled or enabled(except in
/// SPI boot mode). 1: disabled. 0: enabled
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 52, 1);
/// Represents the single-end input threshold vrefh; 1.76 V to 2 V with step of
/// 80 mV
pub const USB_DREFH: EfuseField = EfuseField::new(0, 1, 53, 2);
/// Represents the single-end input threshold vrefl; 1.76 V to 2 V with step of
/// 80 mV
pub const USB_DREFL: EfuseField = EfuseField::new(0, 1, 55, 2);
/// Represents whether the D+ and D- pins is exchanged. 1: exchanged. 0: not
/// exchanged
pub const USB_EXCHG_PINS: EfuseField = EfuseField::new(0, 1, 57, 1);
/// Represents whether vdd spi pin is functioned as gpio. 1: functioned. 0: not
/// functioned
pub const VDD_SPI_AS_GPIO: EfuseField = EfuseField::new(0, 1, 58, 1);
/// Configures the curve of ECDSA calculation: 0: only enable P256. 1: only
/// enable P192. 2: both enable P256 and P192. 3: only enable P256
pub const ECDSA_CURVE_MODE: EfuseField = EfuseField::new(0, 1, 59, 2);
/// Set this bit to permanently turn on ECC const-time mode
pub const ECC_FORCE_CONST_TIME: EfuseField = EfuseField::new(0, 1, 61, 1);
/// Set this bit to control the xts pseudo-round anti-dpa attack function: 0:
/// controlled by register. 1-3: the higher the value is; the more pseudo-rounds
/// are inserted to the xts-aes calculation
pub const XTS_DPA_PSEUDO_LEVEL: EfuseField = EfuseField::new(0, 1, 62, 2);
/// Reserved
pub const RPT4_RESERVED1_1: EfuseField = EfuseField::new(0, 2, 64, 16);
/// Represents whether RTC watchdog timeout threshold is selected at startup. 1:
/// selected. 0: not selected
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 2, 80, 2);
/// Enables flash encryption when 1 or 3 bits are set and disables otherwise
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 2, 82, 3);
/// Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 2, 85, 1);
/// Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 2, 86, 1);
/// Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 2, 87, 1);
/// Represents the purpose of Key0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 2, 88, 4);
/// Represents the purpose of Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 2, 92, 4);
/// Represents the purpose of Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 3, 96, 4);
/// Represents the purpose of Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 3, 100, 4);
/// Represents the purpose of Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 3, 104, 4);
/// Represents the purpose of Key5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 3, 108, 4);
/// Represents the spa secure level by configuring the clock random divide mode
pub const SEC_DPA_LEVEL: EfuseField = EfuseField::new(0, 3, 112, 2);
/// Reserved
pub const RESERVE_0_114: EfuseField = EfuseField::new(0, 3, 114, 1);
/// Represents whether anti-dpa attack is enabled. 1:enabled. 0: disabled
pub const CRYPT_DPA_ENABLE: EfuseField = EfuseField::new(0, 3, 115, 1);
/// Represents whether secure boot is enabled or disabled. 1: enabled. 0:
/// disabled
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 116, 1);
/// Represents whether revoking aggressive secure boot is enabled or disabled.
/// 1: enabled. 0: disabled
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 117, 1);
/// Set these bits to enable power glitch function when chip power on
pub const POWERGLITCH_EN1: EfuseField = EfuseField::new(0, 3, 118, 5);
/// reserved
pub const RESERVED_0_123: EfuseField = EfuseField::new(0, 3, 123, 1);
/// Represents the flash waiting time after power-up; in unit of ms. When the
/// value less than 15; the waiting time is the programmed value. Otherwise; the
/// waiting time is 2 times the programmed value
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 124, 4);
/// Represents whether Download mode is disabled or enabled. 1: disabled. 0:
/// enabled
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Represents whether direct boot mode is disabled or enabled. 1: disabled. 0:
/// enabled
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 4, 129, 1);
/// Set this bit to disable USB-Serial-JTAG print during rom boot
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 4, 130, 1);
/// Reserved
pub const RPT4_RESERVED3_5: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Represents whether the USB-Serial-JTAG download function is disabled or
/// enabled. 1: disabled. 0: enabled
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Represents whether security download is enabled or disabled. 1: enabled. 0:
/// disabled
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Set the default UARTboot message output mode
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 134, 2);
/// Represents whether ROM code is forced to send a resume command during SPI
/// boot. 1: forced. 0:not forced
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 136, 1);
/// Represents the version used by ESP-IDF anti-rollback feature
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 137, 16);
/// Represents whether FAST VERIFY ON WAKE is disabled or enabled when Secure
/// Boot is enabled. 1: disabled. 0: enabled
pub const SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField = EfuseField::new(0, 4, 153, 1);
/// Set bits to enable hysteresis function of PAD0~5
pub const HYS_EN_PAD0: EfuseField = EfuseField::new(0, 4, 154, 6);
/// Set bits to enable hysteresis function of PAD6~27
pub const HYS_EN_PAD1: EfuseField = EfuseField::new(0, 5, 160, 22);
/// Reserved
pub const RPT4_RESERVED4_1: EfuseField = EfuseField::new(0, 5, 182, 2);
/// Reserved
pub const RPT4_RESERVED4_0: EfuseField = EfuseField::new(0, 5, 184, 8);
/// MAC address
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// MAC address
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// Stores the extended bits of MAC address
pub const MAC_EXT: EfuseField = EfuseField::new(1, 1, 48, 16);
/// Stores RF Calibration data. RXIQ version
pub const RXIQ_VERSION: EfuseField = EfuseField::new(1, 2, 64, 3);
/// Stores RF Calibration data. RXIQ data 0
pub const RXIQ_0: EfuseField = EfuseField::new(1, 2, 67, 7);
/// Stores RF Calibration data. RXIQ data 1
pub const RXIQ_1: EfuseField = EfuseField::new(1, 2, 74, 7);
/// Stores the PMU active hp dbias
pub const ACTIVE_HP_DBIAS: EfuseField = EfuseField::new(1, 2, 81, 5);
/// Stores the PMU active lp dbias
pub const ACTIVE_LP_DBIAS: EfuseField = EfuseField::new(1, 2, 86, 5);
/// Stores the PMU sleep dbias
pub const DSLP_DBIAS: EfuseField = EfuseField::new(1, 2, 91, 4);
/// Stores the low 1 bit of dbias_vol_gap
pub const DBIAS_VOL_GAP: EfuseField = EfuseField::new(1, 2, 95, 5);
/// Reserved
pub const MAC_RESERVED_2: EfuseField = EfuseField::new(1, 3, 100, 14);
/// Stores the wafer version minor
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(1, 3, 114, 3);
/// Stores the wafer version major
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 117, 2);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 119, 1);
/// Stores the flash cap
pub const FLASH_CAP: EfuseField = EfuseField::new(1, 3, 120, 3);
/// Stores the flash temp
pub const FLASH_TEMP: EfuseField = EfuseField::new(1, 3, 123, 2);
/// Stores the flash vendor
pub const FLASH_VENDOR: EfuseField = EfuseField::new(1, 3, 125, 3);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 4, 128, 3);
/// reserved
pub const RESERVED_1_131: EfuseField = EfuseField::new(1, 4, 131, 29);
/// Stores the second 32 bits of the zeroth part of system data
pub const SYS_DATA_PART0_2: EfuseField = EfuseField::new(1, 5, 160, 32);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// reserved
pub const RESERVED_2_128: EfuseField = EfuseField::new(2, 4, 128, 2);
/// BLK_VERSION_MINOR of BLOCK2. 1: RF Calibration data in BLOCK1
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(2, 4, 130, 3);
/// BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(2, 4, 133, 2);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(2, 4, 135, 1);
/// Temperature calibration data
pub const TEMP_CALIB: EfuseField = EfuseField::new(2, 4, 136, 9);
/// ADC1 calibration data
pub const ADC1_AVE_INITCODE_ATTEN0: EfuseField = EfuseField::new(2, 4, 145, 10);
/// ADC1 calibration data
pub const ADC1_AVE_INITCODE_ATTEN1: EfuseField = EfuseField::new(2, 4, 155, 10);
/// ADC1 calibration data
pub const ADC1_AVE_INITCODE_ATTEN2: EfuseField = EfuseField::new(2, 5, 165, 10);
/// ADC1 calibration data
pub const ADC1_AVE_INITCODE_ATTEN3: EfuseField = EfuseField::new(2, 5, 175, 10);
/// ADC1 calibration data
pub const ADC1_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(2, 5, 185, 10);
/// ADC1 calibration data
pub const ADC1_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(2, 6, 195, 10);
/// ADC1 calibration data
pub const ADC1_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(2, 6, 205, 10);
/// ADC1 calibration data
pub const ADC1_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(2, 6, 215, 10);
/// ADC1 calibration data
pub const ADC1_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 225, 4);
/// ADC1 calibration data
pub const ADC1_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 229, 4);
/// ADC1 calibration data
pub const ADC1_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 233, 4);
/// ADC1 calibration data
pub const ADC1_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 237, 4);
/// ADC1 calibration data
pub const ADC1_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 241, 4);
/// reserved
pub const RESERVED_2_245: EfuseField = EfuseField::new(2, 7, 245, 11);
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

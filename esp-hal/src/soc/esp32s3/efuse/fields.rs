//! This file was automatically generated, please do not edit it manually!
//!
//! Generated: 2025-04-22 11:33
//! Version:   7127dd097e72bb90d0b790d460993126

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
/// Set this bit to disable Dcache
pub const DIS_DCACHE: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Set this bit to disable Icache in download mode (boot_mode\[3:0\] is 0; 1;
/// 2; 3; 6; 7)
pub const DIS_DOWNLOAD_ICACHE: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Set this bit to disable Dcache in download mode ( boot_mode\[3:0\] is 0; 1;
/// 2; 3; 6; 7)
pub const DIS_DOWNLOAD_DCACHE: EfuseField = EfuseField::new(0, 1, 43, 1);
/// Set this bit to disable the function that forces chip into download mode
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Set this bit to disable USB function
pub const DIS_USB_OTG: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Set this bit to disable CAN function
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Disable app cpu
pub const DIS_APP_CPU: EfuseField = EfuseField::new(0, 1, 47, 1);
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
/// Set this bit to enable external PHY
pub const USB_EXT_PHY_ENABLE: EfuseField = EfuseField::new(0, 1, 58, 1);
/// Bluetooth GPIO signal output security level control
pub const BTLC_GPIO_ENABLE: EfuseField = EfuseField::new(0, 1, 59, 2);
/// SPI regulator switches current limit mode
pub const VDD_SPI_MODECURLIM: EfuseField = EfuseField::new(0, 1, 61, 1);
/// SPI regulator high voltage reference
pub const VDD_SPI_DREFH: EfuseField = EfuseField::new(0, 1, 62, 2);
/// SPI regulator medium voltage reference
pub const VDD_SPI_DREFM: EfuseField = EfuseField::new(0, 2, 64, 2);
/// SPI regulator low voltage reference
pub const VDD_SPI_DREFL: EfuseField = EfuseField::new(0, 2, 66, 2);
/// SPI regulator power up signal
pub const VDD_SPI_XPD: EfuseField = EfuseField::new(0, 2, 68, 1);
/// If VDD_SPI_FORCE is 1; determines VDD_SPI voltage
pub const VDD_SPI_TIEH: EfuseField = EfuseField::new(0, 2, 69, 1);
/// Set this bit and force to use the configuration of eFuse to configure
/// VDD_SPI
pub const VDD_SPI_FORCE: EfuseField = EfuseField::new(0, 2, 70, 1);
/// Set SPI regulator to 0 to configure init\[1:0\]=0
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
pub const RPT4_RESERVED0: EfuseField = EfuseField::new(0, 3, 112, 4);
/// Set this bit to enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 116, 1);
/// Set this bit to enable revoking aggressive secure boot
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 117, 1);
/// Set this bit to disable function of usb switch to jtag in module of usb
/// device
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 3, 118, 1);
/// Set this bit to disable usb device
pub const DIS_USB_SERIAL_JTAG: EfuseField = EfuseField::new(0, 3, 119, 1);
/// Set this bit to enable selection between usb_to_jtag and pad_to_jtag through
/// strapping gpio3 when both reg_dis_usb_jtag and reg_dis_pad_jtag are equal to
/// 0
pub const STRAP_JTAG_SEL: EfuseField = EfuseField::new(0, 3, 120, 1);
/// This bit is used to switch internal PHY and external PHY for USB OTG and USB
/// Device
pub const USB_PHY_SEL: EfuseField = EfuseField::new(0, 3, 121, 1);
/// Sample delay configuration of power glitch
pub const POWER_GLITCH_DSENSE: EfuseField = EfuseField::new(0, 3, 122, 2);
/// Configures flash waiting time after power-up; in unit of ms. If the value is
/// less than 15; the waiting time is the configurable value.  Otherwise; the
/// waiting time is twice the configurable value
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 124, 4);
/// Set this bit to disable download mode (boot_mode\[3:0\] = 0; 1; 2; 3; 6; 7)
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Disable direct boot mode
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 4, 129, 1);
/// USB printing
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 4, 130, 1);
/// Flash ECC mode in ROM
pub const FLASH_ECC_MODE: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Set this bit to disable UART download mode through USB
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Set this bit to enable secure UART download mode
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Set the default UART boot message output mode
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 134, 2);
/// Set default power supply for GPIO33-GPIO37; set when SPI flash is
/// initialized
pub const PIN_POWER_SELECTION: EfuseField = EfuseField::new(0, 4, 136, 1);
/// SPI flash type
pub const FLASH_TYPE: EfuseField = EfuseField::new(0, 4, 137, 1);
/// Set Flash page size
pub const FLASH_PAGE_SIZE: EfuseField = EfuseField::new(0, 4, 138, 2);
/// Set 1 to enable ECC for flash boot
pub const FLASH_ECC_EN: EfuseField = EfuseField::new(0, 4, 140, 1);
/// Set this bit to force ROM code to send a resume command during SPI boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 141, 1);
/// Secure version (used by ESP-IDF anti-rollback feature)
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 142, 16);
/// Set this bit to enable power glitch function
pub const POWERGLITCH_EN: EfuseField = EfuseField::new(0, 4, 158, 1);
/// Set this bit to disable download through USB-OTG
pub const DIS_USB_OTG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 159, 1);
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
/// PSRAM capacity
pub const PSRAM_CAP: EfuseField = EfuseField::new(1, 4, 131, 2);
/// PSRAM temperature
pub const PSRAM_TEMP: EfuseField = EfuseField::new(1, 4, 133, 2);
/// PSRAM vendor
pub const PSRAM_VENDOR: EfuseField = EfuseField::new(1, 4, 135, 2);
/// reserved
pub const RESERVED_1_137: EfuseField = EfuseField::new(1, 4, 137, 4);
/// BLOCK1 K_RTC_LDO
pub const K_RTC_LDO: EfuseField = EfuseField::new(1, 4, 141, 7);
/// BLOCK1 K_DIG_LDO
pub const K_DIG_LDO: EfuseField = EfuseField::new(1, 4, 148, 7);
/// BLOCK1 voltage of rtc dbias20
pub const V_RTC_DBIAS20: EfuseField = EfuseField::new(1, 4, 155, 8);
/// BLOCK1 voltage of digital dbias20
pub const V_DIG_DBIAS20: EfuseField = EfuseField::new(1, 5, 163, 8);
/// BLOCK1 digital dbias when hvt
pub const DIG_DBIAS_HVT: EfuseField = EfuseField::new(1, 5, 171, 5);
/// reserved
pub const RESERVED_1_176: EfuseField = EfuseField::new(1, 5, 176, 3);
/// PSRAM capacity bit 3
pub const PSRAM_CAP_3: EfuseField = EfuseField::new(1, 5, 179, 1);
/// reserved
pub const RESERVED_1_180: EfuseField = EfuseField::new(1, 5, 180, 3);
/// WAFER_VERSION_MINOR most significant bit
pub const WAFER_VERSION_MINOR_HI: EfuseField = EfuseField::new(1, 5, 183, 1);
/// WAFER_VERSION_MAJOR
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 5, 184, 2);
/// ADC2 calibration voltage at atten3
pub const ADC2_CAL_VOL_ATTEN3: EfuseField = EfuseField::new(1, 5, 186, 6);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(2, 4, 128, 2);
/// reserved
pub const RESERVED_2_130: EfuseField = EfuseField::new(2, 4, 130, 2);
/// Temperature calibration data
pub const TEMP_CALIB: EfuseField = EfuseField::new(2, 4, 132, 9);
/// ADC OCode
pub const OCODE: EfuseField = EfuseField::new(2, 4, 141, 8);
/// ADC1 init code at atten0
pub const ADC1_INIT_CODE_ATTEN0: EfuseField = EfuseField::new(2, 4, 149, 8);
/// ADC1 init code at atten1
pub const ADC1_INIT_CODE_ATTEN1: EfuseField = EfuseField::new(2, 4, 157, 6);
/// ADC1 init code at atten2
pub const ADC1_INIT_CODE_ATTEN2: EfuseField = EfuseField::new(2, 5, 163, 6);
/// ADC1 init code at atten3
pub const ADC1_INIT_CODE_ATTEN3: EfuseField = EfuseField::new(2, 5, 169, 6);
/// ADC2 init code at atten0
pub const ADC2_INIT_CODE_ATTEN0: EfuseField = EfuseField::new(2, 5, 175, 8);
/// ADC2 init code at atten1
pub const ADC2_INIT_CODE_ATTEN1: EfuseField = EfuseField::new(2, 5, 183, 6);
/// ADC2 init code at atten2
pub const ADC2_INIT_CODE_ATTEN2: EfuseField = EfuseField::new(2, 5, 189, 6);
/// ADC2 init code at atten3
pub const ADC2_INIT_CODE_ATTEN3: EfuseField = EfuseField::new(2, 6, 195, 6);
/// ADC1 calibration voltage at atten0
pub const ADC1_CAL_VOL_ATTEN0: EfuseField = EfuseField::new(2, 6, 201, 8);
/// ADC1 calibration voltage at atten1
pub const ADC1_CAL_VOL_ATTEN1: EfuseField = EfuseField::new(2, 6, 209, 8);
/// ADC1 calibration voltage at atten2
pub const ADC1_CAL_VOL_ATTEN2: EfuseField = EfuseField::new(2, 6, 217, 8);
/// ADC1 calibration voltage at atten3
pub const ADC1_CAL_VOL_ATTEN3: EfuseField = EfuseField::new(2, 7, 225, 8);
/// ADC2 calibration voltage at atten0
pub const ADC2_CAL_VOL_ATTEN0: EfuseField = EfuseField::new(2, 7, 233, 8);
/// ADC2 calibration voltage at atten1
pub const ADC2_CAL_VOL_ATTEN1: EfuseField = EfuseField::new(2, 7, 241, 7);
/// ADC2 calibration voltage at atten2
pub const ADC2_CAL_VOL_ATTEN2: EfuseField = EfuseField::new(2, 7, 248, 7);
/// reserved
pub const RESERVED_2_255: EfuseField = EfuseField::new(2, 7, 255, 1);
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

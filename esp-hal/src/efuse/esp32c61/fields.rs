//! This file was automatically generated, please do not edit it manually!
//!
//! Generated: 2026-03-05 13:04
//! Version:   d435ade68d90ef96b0522478b2d8ba75

#![allow(clippy::empty_docs)]

use crate::efuse::EfuseField;

/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// Represents whether cache is disabled. 1: Disabled 0: Enabled.
pub const DIS_ICACHE: EfuseField = EfuseField::new(0, 1, 39, 1);
/// Represents whether the function of usb switch to jtag is disabled or
/// enabled. 1: disabled 0: enabled
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 1, 40, 1);
/// Represents whether USB-Serial-JTAG is disabled or enabled. 1: disabled 0:
/// enabled
pub const DIS_USB_SERIAL_JTAG: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Represents whether the function that forces chip into download mode is
/// disabled or enabled. 1: disabled 0: enabled
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Represents whether SPI0 controller during boot_mode_download is disabled or
/// enabled. 1: disabled 0: enabled
pub const SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(0, 1, 43, 1);
/// Represents whether the selection between usb_to_jtag and pad_to_jtag through
/// strapping gpio15 when both EFUSE_DIS_PAD_JTAG and EFUSE_DIS_USB_JTAG are
/// equal to 0 is enabled or disabled. 1: enabled 0: disabled
pub const JTAG_SEL_ENABLE: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Represents whether JTAG is disabled in the hard way(permanently). 1:
/// disabled 0: enabled
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Represents whether flash encrypt function is disabled or enabled(except in
/// SPI boot mode). 1: disabled 0: enabled
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Represents the single-end input threshold vrefh of USB_SERIAL_JTAG PHY; 1.76
/// V to 2 V with step of 80 mV
pub const USB_DREFH: EfuseField = EfuseField::new(0, 1, 47, 2);
/// Represents the single-end input threshold vrefl of USB_SERIAL_JTAG PHY; 1.76
/// V to 2 V with step of 80 mV
pub const USB_DREFL: EfuseField = EfuseField::new(0, 1, 49, 2);
/// Represents whether the D+ and D- pins of USB_SERIAL_JTAG PHY is exchanged.
/// 1: exchanged 0: not exchanged
pub const USB_EXCHG_PINS: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Represents whether vdd spi pin is functioned as gpio. 1: functioned 0: not
/// functioned
pub const VDD_SPI_AS_GPIO: EfuseField = EfuseField::new(0, 1, 52, 1);
/// lp wdt timeout threshold at startup = initial timeout value * (2 ^
/// (EFUSE_WDT_DELAY_SEL + 1))
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 1, 53, 2);
/// Enables flash encryption when 1 or 3 bits are set and disables otherwise
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 1, 55, 3);
/// Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 1, 58, 1);
/// Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 1, 59, 1);
/// Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 1, 60, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_61: EfuseField = EfuseField::new(0, 1, 61, 3);
/// Represents the purpose of Key0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 2, 64, 4);
/// Represents the purpose of Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 2, 68, 4);
/// Represents the purpose of Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 2, 72, 4);
/// Represents the purpose of Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 2, 76, 4);
/// Represents the purpose of Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 2, 80, 4);
/// Represents the purpose of Key5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 2, 84, 4);
/// Represents the spa secure level by configuring the clock random divide mode
pub const SEC_DPA_LEVEL: EfuseField = EfuseField::new(0, 2, 88, 2);
/// Represents whether secure boot is enabled or disabled. 1. Enable 0: Disable
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 2, 90, 1);
/// Represents whether revoking aggressive secure boot is enabled or disabled.
/// 1. Enable 0: Disable
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 2, 91, 1);
/// Represents the flash waiting time after power-up; in unit of ms. When the
/// value less than 15; the waiting time is programmed value. Otherwise; the
/// waiting time is 2 times the programmed value
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 2, 92, 4);
/// Represents whether Download mode is disable or enable. 1. Disable 0: Enable
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 3, 96, 1);
/// Represents whether direct boot mode is disabled or enabled. 1. Disable 0:
/// Enable
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 3, 97, 1);
/// Represents whether print from USB-Serial-JTAG is disabled or enabled. 1.
/// Disable 0: Enable
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 3, 98, 1);
/// Represents whether the USB-Serial-JTAG download function is disabled or
/// enabled. 1: Disable 0: Enable
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 3, 99, 1);
/// Represents whether security download is enabled or disabled. 1: Enable 0:
/// Disable
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 3, 100, 1);
/// Represents the types of UART printing
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 3, 101, 2);
/// Represents whether ROM code is forced to send a resume command during SPI
/// boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 3, 103, 1);
/// Represents the version used by ESP-IDF anti-rollback feature
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 3, 104, 16);
/// Represents whether FAST_VERIFY_ON_WAKE is disable or enable when Secure Boot
/// is enable
pub const SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField = EfuseField::new(0, 3, 120, 1);
/// Set bits to enable hysteresis function of PAD0~27
pub const HYS_EN_PAD: EfuseField = EfuseField::new(0, 3, 121, 1);
/// Represents whether xts-aes anti-dpa attack clock is enabled. 1. Enable. 0:
/// Disable.
pub const XTS_DPA_CLK_ENABLE: EfuseField = EfuseField::new(0, 3, 122, 1);
/// Represents the pseudo round level of xts-aes anti-dpa attack. 3: High. 2:
/// Moderate 1. Low 0: Disabled
pub const XTS_DPA_PSEUDO_LEVEL: EfuseField = EfuseField::new(0, 3, 123, 2);
/// Represents whether the WIFI6 feature is enable or disabled. 1: WIFI6 is
/// disable; 0: WIFI6 is enabled
pub const DIS_WIFI6: EfuseField = EfuseField::new(0, 3, 125, 1);
/// Represents whether to disable P192 curve in ECDSA. 1: Disabled. 0: Not
/// disabled
pub const ECDSA_DISABLE_P192: EfuseField = EfuseField::new(0, 3, 126, 1);
/// Represents whether to force ecc to use const-time calculation mode. 1:
/// Enable. 0: Disable
pub const ECC_FORCE_CONST_TIME: EfuseField = EfuseField::new(0, 3, 127, 1);
/// Represents the anti-rollback secure version of the 2nd stage bootloader used
/// by the ROM bootloader
pub const BOOTLOADER_ANTI_ROLLBACK_SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 128, 4);
/// Represents whether the ani-rollback check for the 2nd stage bootloader is
/// enabled.1: Enabled0: Disabled
pub const BOOTLOADER_ANTI_ROLLBACK_EN: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Represents whether the ani-rollback SECURE_VERSION will be updated from the
/// ROM bootloader.1: Enable0: Disable
pub const BOOTLOADER_ANTI_ROLLBACK_UPDATE_IN_ROM: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Represents the starting flash sector (flash sector size is 0x1000) of the
/// recovery bootloader used by the ROM bootloader If the primary bootloader
/// fails. 0 and 0xFFF - this feature is disabled
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR: EfuseField = EfuseField::new(0, 4, 134, 12);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_146: EfuseField = EfuseField::new(0, 4, 146, 14);
/// Reserved
pub const REPEAT_DATA4: EfuseField = EfuseField::new(0, 5, 160, 24);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_184: EfuseField = EfuseField::new(0, 5, 184, 8);
/// MAC address
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// MAC address
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_1_48: EfuseField = EfuseField::new(1, 1, 48, 16);
/// Minor chip version
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(1, 2, 64, 4);
/// Major chip version
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 68, 2);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 70, 1);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 71, 1);
/// BLK_VERSION_MINOR of BLOCK2
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(1, 2, 72, 3);
/// BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 75, 2);
/// Flash capacity
pub const FLASH_CAP: EfuseField = EfuseField::new(1, 2, 77, 3);
/// Flash vendor
pub const FLASH_VENDOR: EfuseField = EfuseField::new(1, 2, 80, 3);
/// PSRAM capacity
pub const PSRAM_CAP: EfuseField = EfuseField::new(1, 2, 83, 3);
/// PSRAM vendor
pub const PSRAM_VENDOR: EfuseField = EfuseField::new(1, 2, 86, 2);
/// Temperature
pub const TEMP: EfuseField = EfuseField::new(1, 2, 88, 2);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 2, 90, 3);
/// Active HP DBIAS of fixed voltage
pub const ACTIVE_HP_DBIAS: EfuseField = EfuseField::new(1, 2, 93, 4);
/// Active LP DBIAS of fixed voltage
pub const ACTIVE_LP_DBIAS: EfuseField = EfuseField::new(1, 3, 97, 4);
/// LSLP HP DBG of fixed voltage
pub const LSLP_HP_DBG: EfuseField = EfuseField::new(1, 3, 101, 2);
/// LSLP HP DBIAS of fixed voltage
pub const LSLP_HP_DBIAS: EfuseField = EfuseField::new(1, 3, 103, 4);
/// DSLP LP DBG of fixed voltage
pub const DSLP_LP_DBG: EfuseField = EfuseField::new(1, 3, 107, 4);
/// DSLP LP DBIAS of fixed voltage
pub const DSLP_LP_DBIAS: EfuseField = EfuseField::new(1, 3, 111, 5);
/// DBIAS gap between LP and HP
pub const LP_HP_DBIAS_VOL_GAP: EfuseField = EfuseField::new(1, 3, 116, 5);
/// reserved
pub const RESERVED_1_121: EfuseField = EfuseField::new(1, 3, 121, 7);
/// Represents the first 14-bit of zeroth part of system data
pub const SYS_DATA_PART0_1: EfuseField = EfuseField::new(1, 4, 128, 32);
/// Represents the second 32-bit of zeroth part of system data
pub const SYS_DATA_PART0_2: EfuseField = EfuseField::new(1, 5, 160, 32);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// Temperature calibration data
pub const TEMPERATURE_SENSOR: EfuseField = EfuseField::new(2, 4, 128, 9);
/// ADC OCode calibration
pub const OCODE: EfuseField = EfuseField::new(2, 4, 137, 8);
/// Average initcode of ADC1 atten0
pub const ADC1_AVE_INIT_CODE_ATTEN0: EfuseField = EfuseField::new(2, 4, 145, 10);
/// Average initcode of ADC1 atten1
pub const ADC1_AVE_INIT_CODE_ATTEN1: EfuseField = EfuseField::new(2, 4, 155, 10);
/// Average initcode of ADC1 atten2
pub const ADC1_AVE_INIT_CODE_ATTEN2: EfuseField = EfuseField::new(2, 5, 165, 10);
/// Average initcode of ADC1 atten3
pub const ADC1_AVE_INIT_CODE_ATTEN3: EfuseField = EfuseField::new(2, 5, 175, 10);
/// HI_DOUT of ADC1 atten0
pub const ADC1_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(2, 5, 185, 10);
/// HI_DOUT of ADC1 atten1
pub const ADC1_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(2, 6, 195, 10);
/// HI_DOUT of ADC1 atten2
pub const ADC1_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(2, 6, 205, 10);
/// HI_DOUT of ADC1 atten3
pub const ADC1_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(2, 6, 215, 10);
/// Gap between ADC1 CH0 and average initcode
pub const ADC1_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 225, 4);
/// Gap between ADC1 CH1 and average initcode
pub const ADC1_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 229, 4);
/// Gap between ADC1 CH2 and average initcode
pub const ADC1_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 233, 4);
/// Gap between ADC1 CH3 and average initcode
pub const ADC1_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 237, 4);
/// reserved
pub const RESERVED_2_241: EfuseField = EfuseField::new(2, 7, 241, 15);
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

//! eFuse fields for the ESP32-H21.
//!
//! This file was generated from espefuse/efuse_defs/esp32h21.yaml
//! (via espflash/espflash/src/target/efuse/esp32h21.rs).
//!
//! For information on how to regenerate these files, please refer to the
//! `xtask` package's `README.md` file.
use crate::efuse::EfuseField;
/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// Represents whether to enable PVT power glitch monitor function.1: Enable. 0:
/// Disable
pub const PVT_GLITCH_EN: EfuseField = EfuseField::new(0, 1, 39, 1);
/// Represents whether icache is disabled or enabled. 1: Disabled 0: Enabled
pub const DIS_ICACHE: EfuseField = EfuseField::new(0, 1, 40, 1);
/// Represents whether the USB-to-JTAG function in USB Serial/JTAG is disabled.
/// 1: Disabled 0: Enabled
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Represents whether to enable power glitch function.
pub const POWERGLITCH_EN: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Represents whether to disable USB-Serial-JTAG.
pub const DIS_USB_SERIAL_JTAG: EfuseField = EfuseField::new(0, 1, 43, 1);
/// Represents whether the function that forces chip into Download mode is
/// disabled.  1: Disabled 0: Enabled
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Represents accessing MSPI flash/MSPI RAM by SYS AXI matrix is disabled
/// during boot_mode_download. 1: Disabled 0: Enabled
pub const SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Represents whether TWAI function is disabled.  1: Disabled 0: Enabled
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Represents whether the selection of a JTAG signal source through the
/// strapping pin value is enabled when both EFUSE_DIS_PAD_JTAG and
/// EFUSE_DIS_USB_JTAG are configured to 0. For more information; please refer
/// to Chapter Placeholder. 1: Enabled 0: Disabled
pub const JTAG_SEL_ENABLE: EfuseField = EfuseField::new(0, 1, 47, 1);
/// Represents whether PAD JTAG is disabled in the soft way. It can be restarted
/// via HMAC.  Odd count of bits with a value of 1: Disabled Even count of bits
/// with a value of 1: Enabled
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 48, 3);
/// Represents whether PAD JTAG is disabled in the hard way (permanently). 1:
/// Disabled 0: Enabled
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Represents whether flash encryption is disabled (except in SPI boot mode).
/// 1: Disabled 0: Enabled
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 52, 1);
/// Represents single-end input threshold vrefh; 1.76 V to 2 V with step of 80
/// mV; stored in eFuse.
pub const USB_DREFH: EfuseField = EfuseField::new(0, 1, 53, 2);
/// Represents single-end input threshold vrefl; 0.8 V to 1.04 V with step of 80
/// mV; stored in eFuse.
pub const USB_DREFL: EfuseField = EfuseField::new(0, 1, 55, 2);
/// Represents whether the D+ and D- pins is exchanged. 1: Exchanged 0: Not
/// exchanged
pub const USB_EXCHG_PINS: EfuseField = EfuseField::new(0, 1, 57, 1);
/// Represents whether vdd spi pin is functioned as gpio. 1: Functioned 0: Not
/// functioned
pub const VDD_SPI_AS_GPIO: EfuseField = EfuseField::new(0, 1, 58, 1);
/// Represents the configuration of the curve of ECDSA calculation. 0: Only
/// enable P256 1: Only enable P192 2: Both enable P256 and P192 3: Only enable
/// P256
pub const ECDSA_CURVE_MODE: EfuseField = EfuseField::new(0, 1, 59, 2);
/// Represents whether to permanently turn on ECC const-time mode. 0: Disabled
/// 1: Enabled
pub const ECC_FORCE_CONST_TIME: EfuseField = EfuseField::new(0, 1, 61, 1);
/// Represents control method of xts pseudo-round anti-dpa attack function. 0:
/// Controlled by register 1-3: The higher the value is; the more pseudo-rounds
/// are inserted to the xts-aes calculation.
pub const XTS_DPA_PSEUDO_LEVEL: EfuseField = EfuseField::new(0, 1, 62, 2);
/// Represents configuration of IO LDO mode and voltage.
pub const IO_LDO_ADJUST: EfuseField = EfuseField::new(0, 2, 64, 8);
/// Represents configuration of FLASH LDO mode and voltage.
pub const VDD_SPI_LDO_ADJUST: EfuseField = EfuseField::new(0, 2, 72, 8);
/// Represents RTC watchdog timeout threshold. 0: The originally configured STG0
/// threshold x 2 1: The originally configured STG0 threshold x 4 2: The
/// originally configured STG0 threshold x 8 3: The originally configured STG0
/// threshold x 16
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 2, 80, 2);
/// Enables flash encryption when 1 or 3 bits are set and disables otherwise
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 2, 82, 3);
/// Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 2, 85, 1);
/// Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 2, 86, 1);
/// Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 2, 87, 1);
/// Represents the purpose of Key0. See Table tab:efuse-key-purpose.
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 2, 88, 4);
/// Represents the purpose of Key1. See Table tab:efuse-key-purpose.
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 2, 92, 4);
/// Represents the purpose of Key2. See Table tab:efuse-key-purpose.
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 3, 96, 4);
/// Represents the purpose of Key3. See Table tab:efuse-key-purpose.
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 3, 100, 4);
/// Represents the purpose of Key4. See Table tab:efuse-key-purpose.
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 3, 104, 4);
/// Represents the purpose of Key5. See Table tab:efuse-key-purpose.
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 3, 108, 4);
/// Represents the security level of anti-DPA attack. The level is adjusted by
/// configuring the clock random frequency division mode. 0: Security level is
/// SEC_DPA_OFF 1: Security level is SEC_DPA_LOW 2: Security level is
/// SEC_DPA_MIDDLE 3: Security level is SEC_DPA_HIGH For more information;
/// please refer to Chapter mod:sysreg > Section
/// sec:sysreg-anti-dpa-attack-security-control.
pub const SEC_DPA_LEVEL: EfuseField = EfuseField::new(0, 3, 112, 2);
/// Represents select IO LDO voltage to 1.8V or 3.3V. 1: 1.8V 0: 3.3V
pub const IO_LDO_1P8: EfuseField = EfuseField::new(0, 3, 114, 1);
/// Represents whether defense against DPA attack is enabled. 1: Enabled 0:
/// Disabled
pub const CRYPT_DPA_ENABLE: EfuseField = EfuseField::new(0, 3, 115, 1);
/// Represents whether Secure Boot is enabled. 1: Enabled 0: Disabled
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 116, 1);
/// Represents whether aggressive revocation of Secure Boot is enabled. 1:
/// Enabled 0: Disabled
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 117, 1);
/// Represents whether to enable power glitch function when chip power on.
pub const POWERGLITCH_EN1: EfuseField = EfuseField::new(0, 3, 118, 5);
/// Represents whether change DCDC to CCM mode
pub const DCDC_CCM_EN: EfuseField = EfuseField::new(0, 3, 123, 1);
/// Represents the flash waiting time after power-up. Measurement unit: ms. When
/// the value is less than 15; the waiting time is the programmed value.
/// Otherwise; the waiting time is a fixed value; i.e. 30 ms.
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 124, 4);
/// Represents whether all download modes are disabled. 1: Disabled 0: Enabled
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Represents whether direct boot mode is disabled. 1: Disabled 0: Enabled
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 4, 129, 1);
/// Represents whether print from USB-Serial-JTAG during ROM boot is disabled.
/// 1: Disabled 0: Enabled
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 4, 130, 1);
/// Represents whether to select efuse control flash ldo default voltage. 1:
/// efuse0: strapping
pub const FLASH_LDO_EFUSE_SEL: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Represents whether the USB-Serial-JTAG download function is disabled. 1:
/// Disabled 0: Enabled
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Represents whether security download is enabled. Only UART is supported for
/// download. Reading/writing RAM or registers is not supported (i.e. Stub
/// download is not supported). 1: Enabled 0: Disabled
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Set the default UARTboot message output mode
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 134, 2);
/// Represents whether ROM code is forced to send a resume command during SPI
/// boot. 1: Forced 0: Not forced
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 136, 1);
/// Represents the security version used by ESP-IDF anti-rollback feature.
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 137, 16);
/// Represents whether FAST VERIFY ON WAKE is disabled when Secure Boot is
/// enabled. 1: Disabled 0: Enabled
pub const SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField = EfuseField::new(0, 4, 153, 1);
/// Represents whether to enable the hysteresis function of pad 0-5. 0: Disabled
/// 1: Enabled
pub const HYS_EN_PAD0: EfuseField = EfuseField::new(0, 4, 154, 6);
/// Represents whether to enable the hysteresis function of pad 6-27. 0:
/// Disabled 1: Enabled
pub const HYS_EN_PAD1: EfuseField = EfuseField::new(0, 5, 160, 22);
/// Represents which flash LDO is selected. 0: FLASH LDO 1P8. 1: FLASH LDO 1P2.
pub const FLASH_LDO_POWER_SEL: EfuseField = EfuseField::new(0, 5, 182, 1);
/// MAC address
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// MAC address
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// Represents the extended bits of MAC address
pub const MAC_EXT: EfuseField = EfuseField::new(1, 1, 48, 16);
/// Reserved
pub const MAC_RESERVED_0: EfuseField = EfuseField::new(1, 2, 64, 14);
/// Reserved
pub const MAC_RESERVED_1: EfuseField = EfuseField::new(1, 2, 78, 18);
/// Reserved
pub const MAC_RESERVED_2: EfuseField = EfuseField::new(1, 3, 96, 4);
/// Represents the selection of Power glitch monitor PVT cell.
pub const PVT_CELL_SELECT: EfuseField = EfuseField::new(1, 3, 100, 7);
/// Reserved
pub const MAC_RESERVED_3: EfuseField = EfuseField::new(1, 3, 107, 7);
/// Minor version of BLOCK2
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(1, 3, 114, 3);
/// Major version of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 117, 2);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 119, 1);
/// Mark the capacity of in-package Flash
pub const FLASH_CAP: EfuseField = EfuseField::new(1, 3, 120, 3);
/// Mark the vendor of in-package Flash
pub const FLASH_VENDOR: EfuseField = EfuseField::new(1, 3, 123, 3);
/// Mark the specified maximum ambient temperature that ESP Chip can work
/// properly
pub const TEMP: EfuseField = EfuseField::new(1, 3, 126, 2);
/// Represents the threshold of power glitch monitor.
pub const PVT_LIMIT: EfuseField = EfuseField::new(1, 4, 133, 16);
/// Represents whether to trigger reset or charge pump when PVT power glitch
/// happened. 1:Trigger charge pump.  0:Trigger reset
pub const PVT_GLITCH_CHARGE_RESET: EfuseField = EfuseField::new(1, 4, 149, 1);
/// Represents the configuration of glitch mode.
pub const PVT_GLITCH_MODE: EfuseField = EfuseField::new(1, 4, 150, 2);
/// Represents the configuration voltage monitor limit for charge pump.
pub const PVT_PUMP_LIMIT: EfuseField = EfuseField::new(1, 4, 152, 8);
/// Use to configure charge pump voltage gain.
pub const PUMP_DRV: EfuseField = EfuseField::new(1, 5, 160, 4);
/// Minor chip version
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(1, 5, 164, 4);
/// Major chip version
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 5, 168, 2);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 5, 170, 1);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 5, 171, 3);
/// reserved
pub const RESERVED_1_174: EfuseField = EfuseField::new(1, 5, 174, 18);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// Represents the zeroth 32-bit of first part of system data
pub const SYS_DATA_PART1_4: EfuseField = EfuseField::new(2, 4, 128, 32);
/// Represents the zeroth 32-bit of first part of system data
pub const SYS_DATA_PART1_5: EfuseField = EfuseField::new(2, 5, 160, 32);
/// Represents the zeroth 32-bit of first part of system data
pub const SYS_DATA_PART1_6: EfuseField = EfuseField::new(2, 6, 192, 32);
/// Represents the zeroth 32-bit of first part of system data
pub const SYS_DATA_PART1_7: EfuseField = EfuseField::new(2, 7, 224, 32);
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

//! eFuse fields for the ESP32-H4.
//!
//! This file was generated from espefuse/efuse_defs/esp32h4.yaml
//! (via espflash/espflash/src/target/efuse/esp32h4.rs).
//!
//! For information on how to regenerate these files, please refer to the
//! `xtask` package's `README.md` file.
use crate::efuse::EfuseField;
/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// Represents whether the function of usb switch to jtag is disabled or
/// enabled. 1: disabled 0: enabled
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 1, 39, 1);
/// Represents whether USB-Serial-JTAG is disabled or enabled. 1: disabled 0:
/// enabled
pub const DIS_USB_SERIAL_JTAG: EfuseField = EfuseField::new(0, 1, 40, 1);
/// Represents whether the function that forces chip into download mode is
/// disabled or enabled. 1: disabled 0: enabled
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Represents whether SPI0 controller during boot_mode_download is disabled or
/// enabled. 1: disabled 0: enabled
pub const SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Represents whether TWAI function is disabled or enabled. 1: disabled 0:
/// enabled
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 43, 1);
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
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_47: EfuseField = EfuseField::new(0, 1, 47, 3);
/// Represents whether to enable PVT power glitch monitor function.1:Enable.
/// 0:Disable
pub const PVT_GLITCH_EN: EfuseField = EfuseField::new(0, 1, 50, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_51: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Use to configure glitch mode
pub const PVT_GLITCH_MODE: EfuseField = EfuseField::new(0, 1, 52, 2);
/// Represents whether the CPU-Core1 is disabled.  1: Disabled.  0: Not disable
pub const DIS_CORE1: EfuseField = EfuseField::new(0, 1, 54, 1);
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
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 2, 64, 5);
/// Represents the purpose of Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 2, 69, 5);
/// Represents the purpose of Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 2, 74, 5);
/// Represents the purpose of Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 2, 79, 5);
/// Represents the purpose of Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 2, 84, 5);
/// Represents the purpose of Key5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 2, 89, 5);
/// Represents the spa secure level by configuring the clock random divide mode
pub const SEC_DPA_LEVEL: EfuseField = EfuseField::new(0, 2, 94, 2);
/// Represents the pseudo round level of xts-aes anti-dpa attack. 3: High. 2:
/// Moderate 1. Low 0: Disabled
pub const XTS_DPA_PSEUDO_LEVEL: EfuseField = EfuseField::new(0, 3, 96, 2);
/// Represents whether xts-aes anti-dpa attack clock is enabled. 1. Enable. 0:
/// Disable.
pub const XTS_DPA_CLK_ENABLE: EfuseField = EfuseField::new(0, 3, 98, 1);
/// Represents whether to force ecc to use const-time calculation mode.  1:
/// Enable.  0: Disable
pub const ECC_FORCE_CONST_TIME: EfuseField = EfuseField::new(0, 3, 99, 1);
/// Represents if the chip supports Secure Boot using SHA-384
pub const SECURE_BOOT_SHA384_EN: EfuseField = EfuseField::new(0, 3, 100, 1);
/// Represents whether secure boot is enabled or disabled. 1: enabled 0:
/// disabled
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 101, 1);
/// Represents whether revoking aggressive secure boot is enabled or disabled.
/// 1: enabled. 0: disabled
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 102, 1);
/// Represents whether the new key deployment of key manager is disabled. Bit0:
/// Represents whether the new ECDSA key deployment is disabled0: Enabled1:
/// DisabledBit1: Represents whether the new XTS-AES (flash and PSRAM) key
/// deployment is disabled0: Enabled1: DisabledBit2: Represents whether the new
/// HMAC key deployment is disabled0: Enabled1: DisabledBit3: Represents whether
/// the new DS key deployment is disabled0: Enabled1: Disabled
pub const KM_DISABLE_DEPLOY_MODE: EfuseField = EfuseField::new(0, 3, 103, 5);
/// Represents the cycle at which the Key Manager switches random numbers.0:
/// Controlled by the
/// \hyperref\[fielddesc:KEYMNGRNDSWITCHCYCLE\]{KEYMNG\_RND\_SWITCH\_CYCLE}
/// register. For more information; please refer to Chapter \ref{mod:keymng}
/// \textit{\nameref{mod:keymng}}1: 8 Key Manager clock cycles2: 16 Key Manager
/// clock cycles3: 32 Key Manager clock cycles
pub const KM_RND_SWITCH_CYCLE: EfuseField = EfuseField::new(0, 3, 108, 2);
/// Represents whether the corresponding key can be deployed only once.Bit0:
/// Represents whether the ECDSA key can be deployed only once0: The key can be
/// deployed multiple times1: The key can be deployed only onceBit1: Represents
/// whether the XTS-AES (flash and PSRAM) key can be deployed only once0: The
/// key can be deployed multiple times1: The key can be deployed only onceBit2:
/// Represents whether the HMAC key can be deployed only once0: The key can be
/// deployed multiple times1: The key can be deployed only onceBit3: Represents
/// whether the DS key can be deployed only once0: The key can be deployed
/// multiple times1: The key can be deployed only once
pub const KM_DEPLOY_ONLY_ONCE: EfuseField = EfuseField::new(0, 3, 110, 5);
/// Represents whether the corresponding key must come from Key Manager. Bit0:
/// Represents whether the ECDSA key must come from Key Manager.0: The key does
/// not need to come from Key Manager1: The key must come from Key ManagerBit1:
/// Represents whether the XTS-AES (flash and PSRAM) key must come from Key
/// Manager.0: The key does not need to come from Key Manager1: The key must
/// come from Key ManagerBit2: Represents whether the HMAC key must come from
/// Key Manager.0: The key does not need to come from Key Manager1: The key must
/// come from Key ManagerBit3: Represents whether the DS key must come from Key
/// Manager.0: The key does not need to come from Key Manager1: The key must
/// come from Key Manager
pub const FORCE_USE_KEY_MANAGER_KEY: EfuseField = EfuseField::new(0, 3, 115, 5);
/// Represents whether to disable the use of the initialization key written by
/// software and instead force use efuse\_init\_key.0: Enable1: Disable
pub const FORCE_DISABLE_SW_INIT_KEY: EfuseField = EfuseField::new(0, 3, 120, 1);
/// Represents which key flash encryption uses.0: XTS-AES-256 key1: XTS-AES-128
/// key
pub const KM_XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(0, 3, 121, 1);
/// Represents whether the keys in the Key Manager are locked after
/// deployment.0: Not locked1: Locked
pub const LOCK_KM_KEY: EfuseField = EfuseField::new(0, 3, 122, 1);
/// Represents the flash waiting time after power-up; in unit of ms. When the
/// value less than 15; the waiting time is the programmed value. Otherwise; the
/// waiting time is 2 times the programmed value
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 123, 3);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_126: EfuseField = EfuseField::new(0, 3, 126, 1);
/// Represents whether Download mode is disabled or enabled. 1: disabled 0:
/// enabled
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 3, 127, 1);
/// Represents whether direct boot mode is disabled or enabled. 1: disabled 0:
/// enabled
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Represents whether print from USB-Serial-JTAG is disabled or enabled. 1:
/// disabled 0: enabled
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 4, 129, 1);
/// Represents whether the USB-Serial-JTAG download function is disabled or
/// enabled. 1: Disable 0: Enable
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 130, 1);
/// Represents whether security download is enabled or disabled. 1: enabled 0:
/// disabled
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Represents the type of UART printing. 00: force enable printing 01: enable
/// printing when GPIO8 is reset at low level 10: enable printing when GPIO8 is
/// reset at high level 11: force disable printing
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 132, 2);
/// Represents whether ROM code is forced to send a resume command during SPI
/// boot. 1: forced 0:not forced
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 134, 1);
/// Represents the version used by ESP-IDF anti-rollback feature
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 135, 16);
/// Represents whether the HUK generate mode is valid.Odd count of bits with a
/// value of 1: InvalidEven count of bits with a value of 1: Valid
pub const HUK_GEN_STATE: EfuseField = EfuseField::new(0, 4, 151, 5);
/// Represents whether to select efuse control flash ldo default voltage.  1 :
/// efuse 0 : strapping
pub const FLASH_LDO_EFUSE_SEL: EfuseField = EfuseField::new(0, 4, 156, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_157: EfuseField = EfuseField::new(0, 4, 157, 3);
/// Represents the single-end input threshold vrefh of USB_SERIAL_JTAG PHY; 1.76
/// V to 2 V with step of 80 mV
pub const USB_DREFH: EfuseField = EfuseField::new(0, 5, 160, 2);
/// Represents the single-end input threshold vrefl of USB_SERIAL_JTAG PHY; 1.76
/// V to 2 V with step of 80 mV
pub const USB_DREFL: EfuseField = EfuseField::new(0, 5, 162, 2);
/// Represents the single-end input threshold vrefh of USB_OTG_FS PHY; 1.76 V to
/// 2 V with step of 80 mV
pub const USB_OTG_FS_DREFH: EfuseField = EfuseField::new(0, 5, 164, 2);
/// Represents the single-end input threshold vrefl of USB_OTG_FS PHY; 1.76 V to
/// 2 V with step of 80 mV
pub const USB_OTG_FS_DREFL: EfuseField = EfuseField::new(0, 5, 166, 2);
/// Represents whether the D+ and D- pins of USB_SERIAL_JTAG PHY is exchanged.
/// 1: exchanged 0: not exchanged
pub const USB_EXCHG_PINS: EfuseField = EfuseField::new(0, 5, 168, 1);
/// Represents whether the D+ and D- pins of USB_OTG_FS PHY is exchanged. 1:
/// exchanged 0: not exchanged
pub const USB_OTG_FS_EXCHG_PINS: EfuseField = EfuseField::new(0, 5, 169, 1);
/// Represents whether to exchange the USB_SERIAL_JTAG PHY with USB_OTG_FS PHY.
/// 1: exchanged.  0: not exchanged
pub const USB_PHY_SEL: EfuseField = EfuseField::new(0, 5, 170, 1);
/// Represents whether JTAG is disabled in soft way. Odd number: disabled Even
/// number: enabled
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 5, 171, 3);
/// Represents configuration of IO LDO mode and voltage.
pub const IO_LDO_ADJUST: EfuseField = EfuseField::new(0, 5, 174, 8);
/// Represents select IO LDO voltage to 1.8V or 3.3V. 1: 1.8V 0: 3.3V
pub const IO_LDO_1P8: EfuseField = EfuseField::new(0, 5, 182, 1);
/// Represents whether change DCDC to CCM mode
pub const DCDC_CCM_EN: EfuseField = EfuseField::new(0, 5, 183, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_0_184: EfuseField = EfuseField::new(0, 5, 184, 8);
/// MAC address
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// MAC address
pub const MAC1: EfuseField = EfuseField::new(1, 1, 32, 16);
/// Represents the extended bits of MAC address
pub const MAC_EXT: EfuseField = EfuseField::new(1, 1, 48, 16);
/// Power glitch monitor threthold
pub const PVT_LIMIT: EfuseField = EfuseField::new(1, 2, 64, 16);
/// Power glitch monitor PVT cell select
pub const PVT_CELL_SELECT: EfuseField = EfuseField::new(1, 2, 80, 7);
/// Use to configure voltage monitor limit for charge pump
pub const PVT_PUMP_LIMIT: EfuseField = EfuseField::new(1, 2, 87, 8);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_1_95: EfuseField = EfuseField::new(1, 2, 95, 1);
/// Use to configure charge pump voltage gain
pub const PUMP_DRV: EfuseField = EfuseField::new(1, 3, 96, 4);
/// Represents the threshold level of the RTC watchdog STG0 timeout. 0: Original
/// threshold configuration value of STG0 *2 1: Original threshold configuration
/// value of STG0 *4 2: Original threshold configuration value of STG0 *8 3:
/// Original threshold configuration value of STG0 *16
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(1, 3, 100, 2);
/// Represents whether the hysteresis function of corresponding PAD is enabled.
/// 1: enabled 0:disabled
pub const HYS_EN_PAD: EfuseField = EfuseField::new(1, 3, 102, 1);
/// Represents whether to trigger reset or charge pump when PVT power glitch
/// happened.1:Trigger charge pump. 0:Trigger reset
pub const PVT_GLITCH_CHARGE_RESET: EfuseField = EfuseField::new(1, 3, 103, 1);
/// Reserved; it was created by set_missed_fields_in_regs func
pub const RESERVE_1_104: EfuseField = EfuseField::new(1, 3, 104, 1);
/// Represents configuration of FLASH LDO mode and voltage.
pub const VDD_SPI_LDO_ADJUST: EfuseField = EfuseField::new(1, 3, 105, 8);
/// Represents which flash ldo be select: 1: FLASH LDO 1P2 0 : FLASH LDO 1P8
pub const FLASH_LDO_POWER_SEL: EfuseField = EfuseField::new(1, 3, 113, 1);
/// Minor chip version
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(1, 3, 114, 4);
/// Major chip version
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 118, 2);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 120, 1);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 121, 1);
/// BLK_VERSION_MINOR of BLOCK2
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(1, 3, 122, 3);
/// BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 3, 125, 2);
/// Flash capacity
pub const FLASH_CAP: EfuseField = EfuseField::new(1, 3, 127, 3);
/// Flash vendor
pub const FLASH_VENDOR: EfuseField = EfuseField::new(1, 4, 130, 3);
/// Psram capacity
pub const PSRAM_CAP: EfuseField = EfuseField::new(1, 4, 133, 3);
/// Psram vendor
pub const PSRAM_VENDOR: EfuseField = EfuseField::new(1, 4, 136, 2);
/// Temp (die embedded inside)
pub const TEMP: EfuseField = EfuseField::new(1, 4, 138, 2);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 4, 140, 3);
/// PVT DBIAS
pub const PVT_DBIAS: EfuseField = EfuseField::new(1, 4, 143, 5);
/// SPI LDO adjust of 1.2v
pub const ADJUST_1V2: EfuseField = EfuseField::new(1, 4, 148, 4);
/// SPI LDO adjust of 1.8v
pub const ADJUST_1V8: EfuseField = EfuseField::new(1, 4, 152, 4);
/// DCDC-DCDC DBIAS of 1.25v
pub const ACTIVE_DCDC_1V25: EfuseField = EfuseField::new(1, 4, 156, 4);
/// DCDC-DCDC DBIAS of 1.35v
pub const ACTIVE_DCDC_1V35: EfuseField = EfuseField::new(1, 5, 160, 4);
/// DCDC DBIAS in sleep
pub const SLP_DCDC: EfuseField = EfuseField::new(1, 5, 164, 5);
/// HP DRVB in light sleep
pub const LSLP_HP_DRVB: EfuseField = EfuseField::new(1, 5, 169, 5);
/// LP DBIAS in deep sleep
pub const DSLP_LP_DBIAS: EfuseField = EfuseField::new(1, 5, 174, 2);
/// Temperature calibration data
pub const TEMP_CALIB: EfuseField = EfuseField::new(1, 5, 176, 10);
/// reserved
pub const RESERVED_1_186: EfuseField = EfuseField::new(1, 5, 186, 6);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// ADC OCode
pub const OCODE: EfuseField = EfuseField::new(2, 4, 128, 8);
/// DCDC OCode
pub const DCDC_OCODE: EfuseField = EfuseField::new(2, 4, 136, 8);
/// ADC dout of vdd 3.4v
pub const VDD_3V4_DOUT: EfuseField = EfuseField::new(2, 4, 144, 10);
/// Average initcode of ADC1 atten0
pub const ADC1_AVE_INITCODE_ATTEN0: EfuseField = EfuseField::new(2, 4, 154, 9);
/// Average initcode of ADC1 atten1
pub const ADC1_AVE_INITCODE_ATTEN1: EfuseField = EfuseField::new(2, 5, 163, 9);
/// Average initcode of ADC1 atten2
pub const ADC1_AVE_INITCODE_ATTEN2: EfuseField = EfuseField::new(2, 5, 172, 9);
/// Average initcode of ADC1 atten3
pub const ADC1_AVE_INITCODE_ATTEN3: EfuseField = EfuseField::new(2, 5, 181, 9);
/// HI dout of ADC1 atten0
pub const ADC1_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(2, 5, 190, 9);
/// HI dout of ADC1 atten1
pub const ADC1_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(2, 6, 199, 9);
/// HI dout of ADC1 atten2
pub const ADC1_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(2, 6, 208, 9);
/// HI dout of ADC1 atten3
pub const ADC1_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(2, 6, 217, 9);
/// Gap between ADC1 CH0 and average initcode
pub const ADC1_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 226, 3);
/// Gap between ADC1 CH1 and average initcode
pub const ADC1_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 229, 3);
/// Gap between ADC1 CH2 and average initcode
pub const ADC1_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 232, 3);
/// Gap between ADC1 CH3 and average initcode
pub const ADC1_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 235, 3);
/// Gap between ADC1 CH4 and average initcode
pub const ADC1_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(2, 7, 238, 3);
/// Initcode diff between IO LDO 1.8v and 3.3v
pub const INITCODE_DIFF_1P8_3P3: EfuseField = EfuseField::new(2, 7, 241, 5);
/// HI dout diff between IO LDO 1.8v and 3.3v
pub const HI_DOUT_DIFF_1P8_3P3: EfuseField = EfuseField::new(2, 7, 246, 5);
/// reserved
pub const RESERVED_2_251: EfuseField = EfuseField::new(2, 7, 251, 5);
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

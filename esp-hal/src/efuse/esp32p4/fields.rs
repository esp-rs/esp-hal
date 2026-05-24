//! eFuse fields for the ESP32-P4.
//!
//! This file was automatically generated, please do not edit it manually!
//!
//! For information on how to regenerate these files, please refer to the
//! `xtask` package's `README.md` file.
//!
//! Generated on:   2024-03-11
//! ESP-IDF Commit: 0de2912f
use crate::efuse::EfuseField;

/// Disable programming of individual eFuses
pub const WR_DIS: EfuseField = EfuseField::new(0, 0, 0, 32);
/// wr_dis of RD_DIS
pub const WR_DIS_RD_DIS: EfuseField = EfuseField::new(0, 0, 0, 1);
/// wr_dis of KM_RND_SWITCH_CYCLE
pub const WR_DIS_KM_RND_SWITCH_CYCLE: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of KM_DEPLOY_ONLY_ONCE
pub const WR_DIS_KM_DEPLOY_ONLY_ONCE: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of FORCE_USE_KEY_MANAGER_KEY
pub const WR_DIS_FORCE_USE_KEY_MANAGER_KEY: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of FORCE_DISABLE_SW_INIT_KEY
pub const WR_DIS_FORCE_DISABLE_SW_INIT_KEY: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of KM_XTS_KEY_LENGTH_256
pub const WR_DIS_KM_XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of KM_DEPLOY_ONLY_ONCE_H
pub const WR_DIS_KM_DEPLOY_ONLY_ONCE_H: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of FORCE_USE_KEY_MANAGER_KEY_H
pub const WR_DIS_FORCE_USE_KEY_MANAGER_KEY_H: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of LOCK_KM_KEY
pub const WR_DIS_LOCK_KM_KEY: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of KM_DISABLE_DEPLOY_MODE_H
pub const WR_DIS_KM_DISABLE_DEPLOY_MODE_H: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of KM_DISABLE_DEPLOY_MODE
pub const WR_DIS_KM_DISABLE_DEPLOY_MODE: EfuseField = EfuseField::new(0, 0, 1, 1);
/// wr_dis of DIS_USB_JTAG
pub const WR_DIS_DIS_USB_JTAG: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of DIS_FORCE_DOWNLOAD
pub const WR_DIS_DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of SPI_DOWNLOAD_MSPI_DIS
pub const WR_DIS_SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of DIS_TWAI
pub const WR_DIS_DIS_TWAI: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of JTAG_SEL_ENABLE
pub const WR_DIS_JTAG_SEL_ENABLE: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of DIS_PAD_JTAG
pub const WR_DIS_DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of DIS_DOWNLOAD_MANUAL_ENCRYPT
pub const WR_DIS_DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of WDT_DELAY_SEL
pub const WR_DIS_WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of HYS_EN_PAD
pub const WR_DIS_HYS_EN_PAD: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of PXA0_TIEH_SEL_0
pub const WR_DIS_PXA0_TIEH_SEL_0: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of DIS_WDT
pub const WR_DIS_DIS_WDT: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of DIS_SWD
pub const WR_DIS_DIS_SWD: EfuseField = EfuseField::new(0, 0, 2, 1);
/// wr_dis of PVT_GLITCH_EN
pub const WR_DIS_PVT_GLITCH_EN: EfuseField = EfuseField::new(0, 0, 3, 1);
/// wr_dis of PVT_GLITCH_MODE
pub const WR_DIS_PVT_GLITCH_MODE: EfuseField = EfuseField::new(0, 0, 3, 1);
/// wr_dis of SPI_BOOT_CRYPT_CNT
pub const WR_DIS_SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 0, 4, 1);
/// wr_dis of SECURE_BOOT_KEY_REVOKE0
pub const WR_DIS_SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 0, 5, 1);
/// wr_dis of SECURE_BOOT_KEY_REVOKE1
pub const WR_DIS_SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 0, 6, 1);
/// wr_dis of SECURE_BOOT_KEY_REVOKE2
pub const WR_DIS_SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 0, 7, 1);
/// `[WR_DIS.KEY0_PURPOSE]` wr_dis of KEY_PURPOSE_0
pub const WR_DIS_KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 0, 8, 1);
/// wr_dis of KEY_PURPOSE_0_H
pub const WR_DIS_KEY_PURPOSE_0_H: EfuseField = EfuseField::new(0, 0, 8, 1);
/// `[WR_DIS.KEY1_PURPOSE]` wr_dis of KEY_PURPOSE_1
pub const WR_DIS_KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 0, 9, 1);
/// wr_dis of KEY_PURPOSE_1_H
pub const WR_DIS_KEY_PURPOSE_1_H: EfuseField = EfuseField::new(0, 0, 9, 1);
/// `[WR_DIS.KEY2_PURPOSE]` wr_dis of KEY_PURPOSE_2
pub const WR_DIS_KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 0, 10, 1);
/// wr_dis of KEY_PURPOSE_2_H
pub const WR_DIS_KEY_PURPOSE_2_H: EfuseField = EfuseField::new(0, 0, 10, 1);
/// `[WR_DIS.KEY3_PURPOSE]` wr_dis of KEY_PURPOSE_3
pub const WR_DIS_KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 0, 11, 1);
/// wr_dis of KEY_PURPOSE_3_H
pub const WR_DIS_KEY_PURPOSE_3_H: EfuseField = EfuseField::new(0, 0, 11, 1);
/// `[WR_DIS.KEY4_PURPOSE]` wr_dis of KEY_PURPOSE_4
pub const WR_DIS_KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 0, 12, 1);
/// wr_dis of KEY_PURPOSE_4_H
pub const WR_DIS_KEY_PURPOSE_4_H: EfuseField = EfuseField::new(0, 0, 12, 1);
/// `[WR_DIS.KEY5_PURPOSE]` wr_dis of KEY_PURPOSE_5
pub const WR_DIS_KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 0, 13, 1);
/// wr_dis of KEY_PURPOSE_5_H
pub const WR_DIS_KEY_PURPOSE_5_H: EfuseField = EfuseField::new(0, 0, 13, 1);
/// wr_dis of ECC_FORCE_CONST_TIME
pub const WR_DIS_ECC_FORCE_CONST_TIME: EfuseField = EfuseField::new(0, 0, 14, 1);
/// wr_dis of SEC_DPA_LEVEL
pub const WR_DIS_SEC_DPA_LEVEL: EfuseField = EfuseField::new(0, 0, 14, 1);
/// wr_dis of XTS_DPA_CLK_ENABLE
pub const WR_DIS_XTS_DPA_CLK_ENABLE: EfuseField = EfuseField::new(0, 0, 14, 1);
/// wr_dis of XTS_DPA_PSEUDO_LEVEL
pub const WR_DIS_XTS_DPA_PSEUDO_LEVEL: EfuseField = EfuseField::new(0, 0, 14, 1);
/// wr_dis of SECURE_BOOT_EN
pub const WR_DIS_SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 0, 15, 1);
/// wr_dis of SECURE_BOOT_AGGRESSIVE_REVOKE
pub const WR_DIS_SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 0, 16, 1);
/// wr_dis of HP_PWR_SRC_SEL
pub const WR_DIS_HP_PWR_SRC_SEL: EfuseField = EfuseField::new(0, 0, 17, 1);
/// wr_dis of FLASH_ECC_EN
pub const WR_DIS_FLASH_ECC_EN: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of DIS_USB_OTG_DOWNLOAD_MODE
pub const WR_DIS_DIS_USB_OTG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of FLASH_TPUW
pub const WR_DIS_FLASH_TPUW: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of DIS_DOWNLOAD_MODE
pub const WR_DIS_DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of DIS_DIRECT_BOOT
pub const WR_DIS_DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of DIS_USB_SERIAL_JTAG_ROM_PRINT
pub const WR_DIS_DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE
pub const WR_DIS_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of ENABLE_SECURITY_DOWNLOAD
pub const WR_DIS_ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of UART_PRINT_CONTROL
pub const WR_DIS_UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of FORCE_SEND_RESUME
pub const WR_DIS_FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of SECURE_VERSION
pub const WR_DIS_SECURE_VERSION: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of SECURE_BOOT_DISABLE_FAST_WAKE
pub const WR_DIS_SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField = EfuseField::new(0, 0, 18, 1);
/// wr_dis of HUK_GEN_STATE
pub const WR_DIS_HUK_GEN_STATE: EfuseField = EfuseField::new(0, 0, 19, 1);
/// wr_dis of BLOCK1
pub const WR_DIS_BLK1: EfuseField = EfuseField::new(0, 0, 20, 1);
/// `[WR_DIS.MAC_FACTORY]` wr_dis of MAC
pub const WR_DIS_MAC: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of WAFER_VERSION_MINOR
pub const WR_DIS_WAFER_VERSION_MINOR: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of WAFER_VERSION_MAJOR_LO
pub const WR_DIS_WAFER_VERSION_MAJOR_LO: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of DISABLE_WAFER_VERSION_MAJOR
pub const WR_DIS_DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of DISABLE_BLK_VERSION_MAJOR
pub const WR_DIS_DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of BLK_VERSION_MINOR
pub const WR_DIS_BLK_VERSION_MINOR: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of BLK_VERSION_MAJOR
pub const WR_DIS_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of PSRAM_CAP
pub const WR_DIS_PSRAM_CAP: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of TEMP
pub const WR_DIS_TEMP: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of PSRAM_VENDOR
pub const WR_DIS_PSRAM_VENDOR: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of PKG_VERSION
pub const WR_DIS_PKG_VERSION: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of BLOCK2
pub const WR_DIS_SYS_DATA_PART1: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of WAFER_VERSION_MAJOR_HI
pub const WR_DIS_WAFER_VERSION_MAJOR_HI: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO1_DREF
pub const WR_DIS_LDO_VO1_DREF: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO2_DREF
pub const WR_DIS_LDO_VO2_DREF: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO1_MUL
pub const WR_DIS_LDO_VO1_MUL: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO2_MUL
pub const WR_DIS_LDO_VO2_MUL: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO3_K
pub const WR_DIS_LDO_VO3_K: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO3_VOS
pub const WR_DIS_LDO_VO3_VOS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO3_C
pub const WR_DIS_LDO_VO3_C: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO4_K
pub const WR_DIS_LDO_VO4_K: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO4_VOS
pub const WR_DIS_LDO_VO4_VOS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LDO_VO4_C
pub const WR_DIS_LDO_VO4_C: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of ACTIVE_HP_DBIAS
pub const WR_DIS_ACTIVE_HP_DBIAS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of ACTIVE_LP_DBIAS
pub const WR_DIS_ACTIVE_LP_DBIAS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of DSLP_DBG
pub const WR_DIS_DSLP_DBG: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of DSLP_LP_DBIAS
pub const WR_DIS_DSLP_LP_DBIAS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of LP_DCDC_DBIAS_VOL_GAP
pub const WR_DIS_LP_DCDC_DBIAS_VOL_GAP: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of PVT_400M_BIAS
pub const WR_DIS_PVT_400M_BIAS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of PVT_40M_BIAS
pub const WR_DIS_PVT_40M_BIAS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of PVT_100M_BIAS
pub const WR_DIS_PVT_100M_BIAS: EfuseField = EfuseField::new(0, 0, 20, 1);
/// wr_dis of OPTIONAL_UNIQUE_ID
pub const WR_DIS_OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_AVE_INITCODE_ATTEN0
pub const WR_DIS_ADC1_AVE_INITCODE_ATTEN0: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_AVE_INITCODE_ATTEN1
pub const WR_DIS_ADC1_AVE_INITCODE_ATTEN1: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_AVE_INITCODE_ATTEN2
pub const WR_DIS_ADC1_AVE_INITCODE_ATTEN2: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_AVE_INITCODE_ATTEN3
pub const WR_DIS_ADC1_AVE_INITCODE_ATTEN3: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC2_AVE_INITCODE_ATTEN0
pub const WR_DIS_ADC2_AVE_INITCODE_ATTEN0: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC2_AVE_INITCODE_ATTEN1
pub const WR_DIS_ADC2_AVE_INITCODE_ATTEN1: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC2_AVE_INITCODE_ATTEN2
pub const WR_DIS_ADC2_AVE_INITCODE_ATTEN2: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC2_AVE_INITCODE_ATTEN3
pub const WR_DIS_ADC2_AVE_INITCODE_ATTEN3: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_HI_DOUT_ATTEN0
pub const WR_DIS_ADC1_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_HI_DOUT_ATTEN1
pub const WR_DIS_ADC1_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_HI_DOUT_ATTEN2
pub const WR_DIS_ADC1_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(0, 0, 21, 1);
/// wr_dis of ADC1_HI_DOUT_ATTEN3
pub const WR_DIS_ADC1_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(0, 0, 21, 1);
/// `[WR_DIS.USER_DATA]` wr_dis of BLOCK_USR_DATA
pub const WR_DIS_BLOCK_USR_DATA: EfuseField = EfuseField::new(0, 0, 22, 1);
/// `[WR_DIS.MAC_CUSTOM WR_DIS.USER_DATA_MAC_CUSTOM]` wr_dis of CUSTOM_MAC
pub const WR_DIS_CUSTOM_MAC: EfuseField = EfuseField::new(0, 0, 22, 1);
/// `[WR_DIS.KEY0]` wr_dis of BLOCK_KEY0
pub const WR_DIS_BLOCK_KEY0: EfuseField = EfuseField::new(0, 0, 23, 1);
/// `[WR_DIS.KEY1]` wr_dis of BLOCK_KEY1
pub const WR_DIS_BLOCK_KEY1: EfuseField = EfuseField::new(0, 0, 24, 1);
/// `[WR_DIS.KEY2]` wr_dis of BLOCK_KEY2
pub const WR_DIS_BLOCK_KEY2: EfuseField = EfuseField::new(0, 0, 25, 1);
/// `[WR_DIS.KEY3]` wr_dis of BLOCK_KEY3
pub const WR_DIS_BLOCK_KEY3: EfuseField = EfuseField::new(0, 0, 26, 1);
/// `[WR_DIS.KEY4]` wr_dis of BLOCK_KEY4
pub const WR_DIS_BLOCK_KEY4: EfuseField = EfuseField::new(0, 0, 27, 1);
/// `[WR_DIS.KEY5]` wr_dis of BLOCK_KEY5
pub const WR_DIS_BLOCK_KEY5: EfuseField = EfuseField::new(0, 0, 28, 1);
/// `[WR_DIS.SYS_DATA_PART2]` wr_dis of BLOCK_SYS_DATA2
pub const WR_DIS_BLOCK_SYS_DATA2: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_HI_DOUT_ATTEN0
pub const WR_DIS_ADC2_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_HI_DOUT_ATTEN1
pub const WR_DIS_ADC2_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_HI_DOUT_ATTEN2
pub const WR_DIS_ADC2_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_HI_DOUT_ATTEN3
pub const WR_DIS_ADC2_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH0_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH1_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH2_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH3_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH4_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH5_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH5_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH6_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH6_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC1_CH7_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC1_CH7_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_CH0_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC2_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_CH1_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC2_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_CH2_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC2_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_CH3_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC2_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_CH4_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC2_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of ADC2_CH5_ATTEN0_INITCODE_DIFF
pub const WR_DIS_ADC2_CH5_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of TEMPERATURE_SENSOR
pub const WR_DIS_TEMPERATURE_SENSOR: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of USB_DEVICE_EXCHG_PINS
pub const WR_DIS_USB_DEVICE_EXCHG_PINS: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of USB_OTG11_EXCHG_PINS
pub const WR_DIS_USB_OTG11_EXCHG_PINS: EfuseField = EfuseField::new(0, 0, 29, 1);
/// wr_dis of SOFT_DIS_JTAG
pub const WR_DIS_SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 0, 31, 1);
/// Disable reading from BlOCK4-10
pub const RD_DIS: EfuseField = EfuseField::new(0, 1, 32, 7);
/// `[RD_DIS.KEY0]` rd_dis of BLOCK_KEY0
pub const RD_DIS_BLOCK_KEY0: EfuseField = EfuseField::new(0, 1, 32, 1);
/// `[RD_DIS.KEY1]` rd_dis of BLOCK_KEY1
pub const RD_DIS_BLOCK_KEY1: EfuseField = EfuseField::new(0, 1, 33, 1);
/// `[RD_DIS.KEY2]` rd_dis of BLOCK_KEY2
pub const RD_DIS_BLOCK_KEY2: EfuseField = EfuseField::new(0, 1, 34, 1);
/// `[RD_DIS.KEY3]` rd_dis of BLOCK_KEY3
pub const RD_DIS_BLOCK_KEY3: EfuseField = EfuseField::new(0, 1, 35, 1);
/// `[RD_DIS.KEY4]` rd_dis of BLOCK_KEY4
pub const RD_DIS_BLOCK_KEY4: EfuseField = EfuseField::new(0, 1, 36, 1);
/// `[RD_DIS.KEY5]` rd_dis of BLOCK_KEY5
pub const RD_DIS_BLOCK_KEY5: EfuseField = EfuseField::new(0, 1, 37, 1);
/// `[RD_DIS.SYS_DATA_PART2]` rd_dis of BLOCK_SYS_DATA2
pub const RD_DIS_BLOCK_SYS_DATA2: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_HI_DOUT_ATTEN0
pub const RD_DIS_ADC2_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_HI_DOUT_ATTEN1
pub const RD_DIS_ADC2_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_HI_DOUT_ATTEN2
pub const RD_DIS_ADC2_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_HI_DOUT_ATTEN3
pub const RD_DIS_ADC2_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH0_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH1_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH2_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH3_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH4_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH5_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH5_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH6_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH6_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC1_CH7_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC1_CH7_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_CH0_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC2_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_CH1_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC2_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_CH2_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC2_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_CH3_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC2_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_CH4_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC2_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of ADC2_CH5_ATTEN0_INITCODE_DIFF
pub const RD_DIS_ADC2_CH5_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of TEMPERATURE_SENSOR
pub const RD_DIS_TEMPERATURE_SENSOR: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of USB_DEVICE_EXCHG_PINS
pub const RD_DIS_USB_DEVICE_EXCHG_PINS: EfuseField = EfuseField::new(0, 1, 38, 1);
/// rd_dis of USB_OTG11_EXCHG_PINS
pub const RD_DIS_USB_OTG11_EXCHG_PINS: EfuseField = EfuseField::new(0, 1, 38, 1);
/// Represents the starting flash sector (flash sector size is 0x1000) of the recovery
/// bootloader used by the ROM bootloader If the primary bootloader fails. 0 and 0xFFF - this
/// feature is disabled
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR_0_1: EfuseField = EfuseField::new(0, 1, 39, 2);
/// Set this bit to disable function of usb switch to jtag in module of usb device
pub const DIS_USB_JTAG: EfuseField = EfuseField::new(0, 1, 41, 1);
/// Represents the starting flash sector (flash sector size is 0x1000) of the recovery
/// bootloader used by the ROM bootloader If the primary bootloader fails. 0 and 0xFFF - this
/// feature is disabled
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR_2_2: EfuseField = EfuseField::new(0, 1, 42, 1);
/// Set this bit to disable the function that forces chip into download mode
pub const DIS_FORCE_DOWNLOAD: EfuseField = EfuseField::new(0, 1, 44, 1);
/// Set this bit to disable accessing MSPI flash/MSPI ram by SYS AXI matrix during
/// boot_mode_download
pub const SPI_DOWNLOAD_MSPI_DIS: EfuseField = EfuseField::new(0, 1, 45, 1);
/// Set this bit to disable TWAI function
pub const DIS_TWAI: EfuseField = EfuseField::new(0, 1, 46, 1);
/// Set this bit to enable selection between usb_to_jtag and pad_to_jtag through strapping
/// gpio25 when both EFUSE_DIS_PAD_JTAG and EFUSE_DIS_USB_JTAG are equal to 0
pub const JTAG_SEL_ENABLE: EfuseField = EfuseField::new(0, 1, 47, 1);
/// Set odd bits to disable JTAG in the soft way. JTAG can be enabled in HMAC module
pub const SOFT_DIS_JTAG: EfuseField = EfuseField::new(0, 1, 48, 3);
/// Set this bit to disable JTAG in the hard way. JTAG is disabled permanently
pub const DIS_PAD_JTAG: EfuseField = EfuseField::new(0, 1, 51, 1);
/// Set this bit to disable flash manual encrypt function (except in SPI boot mode)
pub const DIS_DOWNLOAD_MANUAL_ENCRYPT: EfuseField = EfuseField::new(0, 1, 52, 1);
/// Represents the starting flash sector (flash sector size is 0x1000) of the recovery
/// bootloader used by the ROM bootloader If the primary bootloader fails. 0 and 0xFFF - this
/// feature is disabled
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR_3_6: EfuseField = EfuseField::new(0, 1, 53, 4);
/// 0: intphy(gpio24/25) <---> usb_device 1: intphy(26/27) <---> usb_otg11.1: intphy(gpio26/27)
/// <---> usb_device 1: intphy(24/25) <---> usb_otg11
pub const USB_PHY_SEL: EfuseField = EfuseField::new(0, 1, 57, 1);
/// Set the bits to control validation of HUK generate mode. Odd of 1 is invalid; even of 1 is
/// valid
pub const HUK_GEN_STATE: EfuseField = EfuseField::new(0, 1, 58, 5);
/// Represents the starting flash sector (flash sector size is 0x1000) of the recovery
/// bootloader used by the ROM bootloader If the primary bootloader fails. 0 and 0xFFF - this
/// feature is disabled
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR_7_7: EfuseField = EfuseField::new(0, 1, 63, 1);
/// Represents the starting flash sector (flash sector size is 0x1000) of the recovery
/// bootloader used by the ROM bootloader If the primary bootloader fails. 0 and 0xFFF - this
/// feature is disabled
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR_8_10: EfuseField = EfuseField::new(0, 2, 64, 3);
/// Represents the starting flash sector (flash sector size is 0x1000) of the recovery
/// bootloader used by the ROM bootloader If the primary bootloader fails. 0 and 0xFFF - this
/// feature is disabled
pub const RECOVERY_BOOTLOADER_FLASH_SECTOR_11_11: EfuseField = EfuseField::new(0, 2, 67, 1);
/// Set the bits to control key manager random number switch cycle. 0: control by register. 1:
/// 8 km clk cycles. 2: 16 km cycles. 3: 32 km cycles
pub const KM_RND_SWITCH_CYCLE: EfuseField = EfuseField::new(0, 2, 68, 1);
/// EFUSE_KM_DEPLOY_ONLY_ONCE and EFUSE_KM_DEPLOY_ONLY_ONCE_H together form one field:
/// {EFUSE_KM_DEPLOY_ONLY_ONCE_H; EFUSE_KM_DEPLOY_ONLY_ONCE`[3:0]`}. Set each bit to control whether
/// corresponding key can only be deployed once. 1 is true; 0 is false. bit 0: ecsda; bit 1: xts;
/// bit2: hmac; bit3: ds; bit4:psram
pub const KM_DEPLOY_ONLY_ONCE: EfuseField = EfuseField::new(0, 3, 118, 5);
/// EFUSE_FORCE_USE_KEY_MANAGER_KEY and EFUSE_FORCE_USE_KEY_MANAGER_KEY_H together form one
/// field: {EFUSE_FORCE_USE_KEY_MANAGER_KEY_H; EFUSE_FORCE_USE_KEY_MANAGER_KEY`[3:0]`}. Set each bit
/// to control whether corresponding key must come from key manager. 1 is true; 0 is false. bit 0:
/// ecsda; bit 1: xts; bit2: hmac; bit3: ds; bit4:psram
pub const FORCE_USE_KEY_MANAGER_KEY: EfuseField = EfuseField::new(0, 3, 119, 5);
/// Set this bit to disable software written init key; and force use efuse_init_key
pub const FORCE_DISABLE_SW_INIT_KEY: EfuseField = EfuseField::new(0, 2, 77, 1);
/// Set this bit to config flash encryption xts-512 key; else use xts-256 key when using the
/// key manager
pub const KM_XTS_KEY_LENGTH_256: EfuseField = EfuseField::new(0, 2, 78, 1);
/// Set this bit to permanently turn on ECC const-time mode
pub const ECC_FORCE_CONST_TIME: EfuseField = EfuseField::new(0, 2, 79, 1);
/// Select lp wdt timeout threshold at startup = initial timeout value * (2 ^
/// (EFUSE_WDT_DELAY_SEL + 1))
pub const WDT_DELAY_SEL: EfuseField = EfuseField::new(0, 2, 81, 1);
/// Set this bit to enable SPI boot encrypt/decrypt. Odd number of 1: enable. even number of 1:
/// disable {0: "Disable"; 1: "Enable"; 3: "Disable"; 7: "Enable"}
pub const SPI_BOOT_CRYPT_CNT: EfuseField = EfuseField::new(0, 2, 82, 3);
/// Revoke 1st secure boot key
pub const SECURE_BOOT_KEY_REVOKE0: EfuseField = EfuseField::new(0, 2, 85, 1);
/// Revoke 2nd secure boot key
pub const SECURE_BOOT_KEY_REVOKE1: EfuseField = EfuseField::new(0, 2, 86, 1);
/// Revoke 3rd secure boot key
pub const SECURE_BOOT_KEY_REVOKE2: EfuseField = EfuseField::new(0, 2, 87, 1);
/// `[KEY0_PURPOSE]` Purpose of Key0
pub const KEY_PURPOSE_0: EfuseField = EfuseField::new(0, 4, 155, 5);
/// `[KEY1_PURPOSE]` Purpose of Key1
pub const KEY_PURPOSE_1: EfuseField = EfuseField::new(0, 4, 156, 5);
/// `[KEY2_PURPOSE]` Purpose of Key2
pub const KEY_PURPOSE_2: EfuseField = EfuseField::new(0, 4, 157, 5);
/// `[KEY3_PURPOSE]` Purpose of Key3
pub const KEY_PURPOSE_3: EfuseField = EfuseField::new(0, 4, 158, 5);
/// `[KEY4_PURPOSE]` Purpose of Key4
pub const KEY_PURPOSE_4: EfuseField = EfuseField::new(0, 4, 159, 5);
/// `[KEY5_PURPOSE]` Purpose of Key5
pub const KEY_PURPOSE_5: EfuseField = EfuseField::new(0, 5, 164, 5);
/// Configures the clock random divide mode to determine the dpa secure level
pub const SEC_DPA_LEVEL: EfuseField = EfuseField::new(0, 3, 112, 2);
/// Sets this bit to enable xts clock anti-dpa attack function
pub const XTS_DPA_CLK_ENABLE: EfuseField = EfuseField::new(0, 3, 115, 1);
/// Set this bit to enable secure boot
pub const SECURE_BOOT_EN: EfuseField = EfuseField::new(0, 3, 116, 1);
/// Set this bit to enable revoking aggressive secure boot
pub const SECURE_BOOT_AGGRESSIVE_REVOKE: EfuseField = EfuseField::new(0, 3, 117, 1);
/// Set this bit to enable ECC for flash boot
pub const FLASH_ECC_EN: EfuseField = EfuseField::new(0, 3, 122, 1);
/// Set this bit to disable download via USB-OTG
pub const DIS_USB_OTG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 3, 123, 1);
/// Configures flash waiting time after power-up; in unit of ms. When the value less than 15;
/// the waiting time is the configurable value. Otherwise; the waiting time is 30
pub const FLASH_TPUW: EfuseField = EfuseField::new(0, 3, 124, 4);
/// Set this bit to disable download mode (boot_mode`[3:0]` = 0; 1; 2; 4; 5; 6; 7)
pub const DIS_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 128, 1);
/// Set this bit to disable direct boot mode
pub const DIS_DIRECT_BOOT: EfuseField = EfuseField::new(0, 4, 129, 1);
/// Set this bit to disable USB-Serial-JTAG print during rom boot
pub const DIS_USB_SERIAL_JTAG_ROM_PRINT: EfuseField = EfuseField::new(0, 4, 130, 1);
/// set this bit to lock the key manager key after deploy
pub const LOCK_KM_KEY: EfuseField = EfuseField::new(0, 4, 131, 1);
/// Set this bit to disable the USB-Serial-JTAG download function
pub const DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE: EfuseField = EfuseField::new(0, 4, 132, 1);
/// Set this bit to enable security download mode
pub const ENABLE_SECURITY_DOWNLOAD: EfuseField = EfuseField::new(0, 4, 133, 1);
/// Set the type of UART printing; 00: force enable printing; 01: enable printing when GPIO8 is
/// reset at low level; 10: enable printing when GPIO8 is reset at high level; 11: force disable
/// printing
pub const UART_PRINT_CONTROL: EfuseField = EfuseField::new(0, 4, 134, 2);
/// Set this bit to force ROM code to send a resume command during SPI boot
pub const FORCE_SEND_RESUME: EfuseField = EfuseField::new(0, 4, 136, 1);
/// Secure version used by ESP-IDF anti-rollback feature
pub const SECURE_VERSION: EfuseField = EfuseField::new(0, 4, 137, 16);
/// Represents whether secure boot do fast verification on wake is disabled. 0: enabled 1:
/// disabled
pub const SECURE_BOOT_DISABLE_FAST_WAKE: EfuseField = EfuseField::new(0, 4, 153, 1);
/// Set bits to enable hysteresis function of PAD0~27
pub const HYS_EN_PAD: EfuseField = EfuseField::new(0, 4, 154, 1);
/// Output LDO VO0 tieh source select. 0: 1'b1 1: sdmmc1 2: reg 3:sdmmc0
pub const PXA0_TIEH_SEL_0: EfuseField = EfuseField::new(0, 5, 160, 2);
/// Represents whether to enable PVT power glitch monitor function.1:Enable. 0:Disable
pub const PVT_GLITCH_EN: EfuseField = EfuseField::new(0, 5, 162, 1);
/// EFUSE_KM_DISABLE_DEPLOY_MODE and EFUSE_KM_DISABLE_DEPLOY_MODE_H together form one field:
/// {EFUSE_KM_DISABLE_DEPLOY_MODE_H; EFUSE_KM_DISABLE_DEPLOY_MODE`[3:0]`}. Set each bit to control
/// whether corresponding key's deploy mode of new value deployment is disabled. 1 is true; 0 is
/// false. bit 0: ecsda; bit 1: xts; bit2: hmac; bit3: ds; bit4:psram
pub const KM_DISABLE_DEPLOY_MODE: EfuseField = EfuseField::new(0, 5, 167, 5);
/// Sets this bit to control the xts pseudo-round anti-dpa attack function. 0: controlled by
/// register. 1-3: the higher the value is; the more pseudo-rounds are inserted to the xts-aes
/// calculation
pub const XTS_DPA_PSEUDO_LEVEL: EfuseField = EfuseField::new(0, 5, 176, 2);
/// HP system power source select. 0:LDO  1: DCDC
pub const HP_PWR_SRC_SEL: EfuseField = EfuseField::new(0, 5, 178, 1);
/// Represents whether secure boot using SHA-384 is enabled. 0: disable 1: enable
pub const SECURE_BOOT_SHA384_EN: EfuseField = EfuseField::new(0, 5, 179, 1);
/// Set this bit to disable watch dog
pub const DIS_WDT: EfuseField = EfuseField::new(0, 5, 180, 1);
/// Set bit to disable super-watchdog
pub const DIS_SWD: EfuseField = EfuseField::new(0, 5, 181, 1);
/// Use to configure glitch mode
pub const PVT_GLITCH_MODE: EfuseField = EfuseField::new(0, 5, 182, 2);
/// `[MAC_FACTORY]` MAC address (low 32 bits)
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
/// `[MAC_FACTORY]` MAC address (high 16 bits)
pub const MAC1: EfuseField = EfuseField::new(1, 0, 32, 16);
/// Minor chip version
pub const WAFER_VERSION_MINOR: EfuseField = EfuseField::new(1, 2, 64, 4);
/// Major chip version (lower 2 bits)
pub const WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 87, 3);
/// Disables check of wafer version major
pub const DISABLE_WAFER_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 70, 1);
/// Disables check of blk version major
pub const DISABLE_BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 71, 1);
/// BLK_VERSION_MINOR of BLOCK2
pub const BLK_VERSION_MINOR: EfuseField = EfuseField::new(1, 2, 72, 3);
/// BLK_VERSION_MAJOR of BLOCK2
pub const BLK_VERSION_MAJOR: EfuseField = EfuseField::new(1, 2, 75, 2);
/// PSRAM capacity
pub const PSRAM_CAP: EfuseField = EfuseField::new(1, 2, 77, 3);
/// Operating temperature of the ESP chip
pub const TEMP: EfuseField = EfuseField::new(1, 2, 80, 2);
/// PSRAM vendor
pub const PSRAM_VENDOR: EfuseField = EfuseField::new(1, 2, 82, 2);
/// Package version
pub const PKG_VERSION: EfuseField = EfuseField::new(1, 2, 84, 3);
/// Output VO1 parameter
pub const LDO_VO1_DREF: EfuseField = EfuseField::new(1, 2, 88, 4);
/// Output VO2 parameter
pub const LDO_VO2_DREF: EfuseField = EfuseField::new(1, 2, 92, 4);
/// Output VO1 parameter
pub const LDO_VO1_MUL: EfuseField = EfuseField::new(1, 3, 96, 3);
/// Output VO2 parameter
pub const LDO_VO2_MUL: EfuseField = EfuseField::new(1, 3, 99, 3);
/// Output VO3 calibration parameter
pub const LDO_VO3_K: EfuseField = EfuseField::new(1, 3, 102, 8);
/// Output VO3 calibration parameter
pub const LDO_VO3_VOS: EfuseField = EfuseField::new(1, 3, 110, 6);
/// Output VO3 calibration parameter
pub const LDO_VO3_C: EfuseField = EfuseField::new(1, 3, 116, 6);
/// Output VO4 calibration parameter
pub const LDO_VO4_K: EfuseField = EfuseField::new(1, 3, 122, 8);
/// Output VO4 calibration parameter
pub const LDO_VO4_VOS: EfuseField = EfuseField::new(1, 4, 130, 6);
/// Output VO4 calibration parameter
pub const LDO_VO4_C: EfuseField = EfuseField::new(1, 4, 136, 6);
/// Active HP DBIAS of fixed voltage
pub const ACTIVE_HP_DBIAS: EfuseField = EfuseField::new(1, 4, 144, 4);
/// Active LP DBIAS of fixed voltage
pub const ACTIVE_LP_DBIAS: EfuseField = EfuseField::new(1, 4, 148, 4);
/// DSLP BDG of fixed voltage
pub const DSLP_DBG: EfuseField = EfuseField::new(1, 4, 156, 4);
/// DSLP LP DBIAS of fixed voltage
pub const DSLP_LP_DBIAS: EfuseField = EfuseField::new(1, 5, 160, 5);
/// DBIAS gap between LP and DCDC
pub const LP_DCDC_DBIAS_VOL_GAP: EfuseField = EfuseField::new(1, 5, 165, 5);
/// PVT_DCM_VSET when the CPU is at 400M
pub const PVT_400M_BIAS: EfuseField = EfuseField::new(1, 5, 171, 5);
/// PVT_DCM_VSET corresponding to about 0.9V fixed voltage when the CPU is at 40M
pub const PVT_40M_BIAS: EfuseField = EfuseField::new(1, 5, 176, 5);
/// PVT_DCM_VSET corresponding to about 1.0V fixed voltage when the CPU is at 100M
pub const PVT_100M_BIAS: EfuseField = EfuseField::new(1, 5, 181, 5);
/// Optional unique 128-bit ID
pub const OPTIONAL_UNIQUE_ID: EfuseField = EfuseField::new(2, 0, 0, 128);
/// Average initcode of ADC1 atten0
pub const ADC1_AVE_INITCODE_ATTEN0: EfuseField = EfuseField::new(2, 4, 128, 10);
/// Average initcode of ADC1 atten1
pub const ADC1_AVE_INITCODE_ATTEN1: EfuseField = EfuseField::new(2, 4, 138, 10);
/// Average initcode of ADC1 atten2
pub const ADC1_AVE_INITCODE_ATTEN2: EfuseField = EfuseField::new(2, 4, 148, 10);
/// Average initcode of ADC1 atten3
pub const ADC1_AVE_INITCODE_ATTEN3: EfuseField = EfuseField::new(2, 4, 158, 10);
/// Average initcode of ADC2 atten0
pub const ADC2_AVE_INITCODE_ATTEN0: EfuseField = EfuseField::new(2, 5, 168, 10);
/// Average initcode of ADC2 atten1
pub const ADC2_AVE_INITCODE_ATTEN1: EfuseField = EfuseField::new(2, 5, 178, 10);
/// Average initcode of ADC2 atten2
pub const ADC2_AVE_INITCODE_ATTEN2: EfuseField = EfuseField::new(2, 5, 188, 10);
/// Average initcode of ADC2 atten3
pub const ADC2_AVE_INITCODE_ATTEN3: EfuseField = EfuseField::new(2, 6, 198, 10);
/// HI_DOUT of ADC1 atten0
pub const ADC1_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(2, 6, 208, 10);
/// HI_DOUT of ADC1 atten1
pub const ADC1_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(2, 6, 218, 10);
/// HI_DOUT of ADC1 atten2
pub const ADC1_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(2, 7, 228, 10);
/// HI_DOUT of ADC1 atten3
pub const ADC1_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(2, 7, 238, 10);
/// `[BLOCK_USR_DATA]` User data
pub const USER_DATA: EfuseField = EfuseField::new(3, 0, 0, 256);
/// `[MAC_CUSTOM CUSTOM_MAC]` Custom MAC
pub const USER_DATA_MAC_CUSTOM: EfuseField = EfuseField::new(3, 6, 200, 48);
/// `[BLOCK_KEY0]` Key0 or user data
pub const KEY0: EfuseField = EfuseField::new(4, 0, 0, 256);
/// `[BLOCK_KEY1]` Key1 or user data
pub const KEY1: EfuseField = EfuseField::new(5, 0, 0, 256);
/// `[BLOCK_KEY2]` Key2 or user data
pub const KEY2: EfuseField = EfuseField::new(6, 0, 0, 256);
/// `[BLOCK_KEY3]` Key3 or user data
pub const KEY3: EfuseField = EfuseField::new(7, 0, 0, 256);
/// `[BLOCK_KEY4]` Key4 or user data
pub const KEY4: EfuseField = EfuseField::new(8, 0, 0, 256);
/// `[BLOCK_KEY5]` Key5 or user data
pub const KEY5: EfuseField = EfuseField::new(9, 0, 0, 256);
/// HI_DOUT of ADC2 atten0
pub const ADC2_HI_DOUT_ATTEN0: EfuseField = EfuseField::new(10, 0, 0, 10);
/// HI_DOUT of ADC2 atten1
pub const ADC2_HI_DOUT_ATTEN1: EfuseField = EfuseField::new(10, 0, 10, 10);
/// HI_DOUT of ADC2 atten2
pub const ADC2_HI_DOUT_ATTEN2: EfuseField = EfuseField::new(10, 0, 20, 10);
/// HI_DOUT of ADC2 atten3
pub const ADC2_HI_DOUT_ATTEN3: EfuseField = EfuseField::new(10, 0, 30, 10);
/// Gap between ADC1_ch0 and average initcode
pub const ADC1_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 1, 40, 4);
/// Gap between ADC1_ch1 and average initcode
pub const ADC1_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 1, 44, 4);
/// Gap between ADC1_ch2 and average initcode
pub const ADC1_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 1, 48, 4);
/// Gap between ADC1_ch3 and average initcode
pub const ADC1_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 1, 52, 4);
/// Gap between ADC1_ch4 and average initcode
pub const ADC1_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 1, 56, 4);
/// Gap between ADC1_ch5 and average initcode
pub const ADC1_CH5_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 1, 60, 4);
/// Gap between ADC1_ch6 and average initcode
pub const ADC1_CH6_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 64, 4);
/// Gap between ADC1_ch7 and average initcode
pub const ADC1_CH7_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 68, 4);
/// Gap between ADC2_ch0 and average initcode
pub const ADC2_CH0_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 72, 4);
/// Gap between ADC2_ch1 and average initcode
pub const ADC2_CH1_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 76, 4);
/// Gap between ADC2_ch2 and average initcode
pub const ADC2_CH2_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 80, 4);
/// Gap between ADC2_ch3 and average initcode
pub const ADC2_CH3_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 84, 4);
/// Gap between ADC2_ch4 and average initcode
pub const ADC2_CH4_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 88, 4);
/// Gap between ADC2_ch5 and average initcode
pub const ADC2_CH5_ATTEN0_INITCODE_DIFF: EfuseField = EfuseField::new(10, 2, 92, 4);
/// Temperature calibration data
pub const TEMPERATURE_SENSOR: EfuseField = EfuseField::new(10, 3, 96, 10);
/// Enable usb device exchange pins of D+ and D-
pub const USB_DEVICE_EXCHG_PINS: EfuseField = EfuseField::new(10, 7, 228, 1);
/// Enable usb otg11 exchange pins of D+ and D-
pub const USB_OTG11_EXCHG_PINS: EfuseField = EfuseField::new(10, 7, 229, 1);

//! # Reading of eFuses (ESP32-S2)
//!
//! ## Overview
//!
//! The `efuse` module provides functionality for reading eFuse data
//! from the `ESP32-S2` chip, allowing access to various chip-specific
//! information such as:
//!
//!   * MAC address
//!   * core count
//!   * CPU frequency
//!   * chip type
//!
//! and more. It is useful for retrieving chip-specific configuration and
//! identification data during runtime.
//!
//! The `Efuse` struct represents the eFuse peripheral and is responsible for
//! reading various eFuse fields and values.
//!
//! ## Examples
//!
//! ### Read data from the eFuse storage.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::efuse::Efuse;
//!
//! let mac_address = Efuse::read_base_mac_address();
//!
//! println!(
//!     "MAC: {:#X}:{:#X}:{:#X}:{:#X}:{:#X}:{:#X}",
//!     mac_address[0],
//!     mac_address[1],
//!     mac_address[2],
//!     mac_address[3],
//!     mac_address[4],
//!     mac_address[5]
//! );
//!
//! println!("MAC address {:02x?}", Efuse::mac_address());
//! println!("Flash Encryption {:?}", Efuse::flash_encryption());
//! # Ok(())
//! # }
//! ```

use self::adc_raw_map::*;
pub use self::fields::*;
use crate::{analog::adc::Attenuation, peripherals::EFUSE, soc::efuse_field::EfuseField};

mod adc_raw_map;
mod fields;

/// A struct representing the eFuse functionality of the chip.
pub struct Efuse;

enum RtcCalibParam {
    V1VLow,
    V1VHigh,
    V2VInit,
    V2VHigh,
}

impl Into<usize> for RtcCalibParam {
    fn into(self) -> usize {
        match self {
            RtcCalibParam::V1VLow => 0,
            RtcCalibParam::V1VHigh => 1,
            RtcCalibParam::V2VInit => 1,
            RtcCalibParam::V2VHigh => 0,
        }
    }
}

const RTC_CALIB_V_HIGH: [i32; 4] = [600, 800, 1000, 2000];
const RTC_CALIB_V_LOW: i32 = 250;

impl Efuse {
    /// Reads chip's MAC address from the eFuse storage.
    pub fn read_base_mac_address() -> [u8; 6] {
        Self::read_field_be(MAC)
    }

    /// Get status of SPI boot encryption.
    pub fn flash_encryption() -> bool {
        (Self::read_field_le::<u8>(SPI_BOOT_CRYPT_CNT).count_ones() % 2) != 0
    }

    /// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
    pub fn rwdt_multiplier() -> u8 {
        Self::read_field_le::<u8>(WDT_DELAY_SEL)
    }

    /// Get efuse block version
    ///
    /// see <https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/efuse/esp32s2/esp_efuse_rtc_table.c#L94>
    pub fn block_version() -> (u8, u8) {
        // See <https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/hal/esp32s2/include/hal/efuse_ll.h#L65-73>
        (
            Self::read_field_le::<u8>(BLK_VERSION_MAJOR),
            Self::read_field_le::<u8>(BLK_VERSION_MINOR),
        )
    }

    /// See <https://github.com/espressif/esp-idf/blob/903af13e847cd301e476d8b16b4ee1c21b30b5c6/components/esp_adc/deprecated/esp32s2/esp_adc_cal_legacy.c#L150>
    pub fn adc_cal_check_efuse() -> bool {
        let (_, calib_version) = Self::block_version();
        let encoding_version = calib_version;

        encoding_version == 1 || encoding_version == 2
    }

    /// See <https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/efuse/esp32s2/esp_efuse_rtc_table.c#L97>
    fn rtc_table_get_tag(
        version: u8,
        unit: u8,
        atten: Attenuation,
        tag: RtcCalibParam,
    ) -> Option<usize> {
        if unit >= 2 {
            return None;
        }

        use RtcCalibParam::*;
        let param_offset = match (version, tag) {
            (1, V1VLow) => TAG_RTCCALIB_V1IDX_A10L,
            (1, V1VHigh) => TAG_RTCCALIB_V1IDX_A10H,
            (2, V2VInit) => TAG_RTCCALIB_V2IDX_A10I,
            (2, V2VHigh) => TAG_RTCCALIB_V2IDX_A10H,
            _ => return None,
        };

        const RTCCALIB_ATTENCOUNT: usize = 4;
        Some(param_offset + (unit as usize) * RTCCALIB_ATTENCOUNT + (atten as usize))
    }

    fn rtc_table_get_raw_efuse_value(tag: usize) -> i32 {
        if tag == 0 || tag >= RAW_MAP.len() {
            return 0;
        }
        fn signed_bit_to_int(number: u32, length: u16) -> i32 {
            if number >> (length - 1) != 0 {
                -((number ^ (1 << (length - 1))) as i32)
            } else {
                number as i32
            }
        }
        let info = &RAW_MAP[tag];
        let val = Self::read_field_le(EfuseField::new(info.block, info.begin_bit, info.length));
        signed_bit_to_int(val, info.length)
    }

    /// See <https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/efuse/esp32s2/esp_efuse_rtc_table.c#L145>
    pub fn rtc_table_get_parsed_efuse_value(tag: usize) -> i32 {
        if tag == 0 || tag >= RAW_MAP.len() {
            return 0;
        }

        let info = &RAW_MAP[tag];
        let efuse_val = Self::rtc_table_get_raw_efuse_value(tag) * info.multiplier;
        efuse_val + info.base + Self::rtc_table_get_raw_efuse_value(info.dependency)
    }

    /// Calculates the coeff_b from the calculate_characterization_coefficient
    pub fn rtc_calib_coeff_b(adcn: usize, atten: Attenuation) -> Option<i32> {
        if adcn > 2 || adcn == 0 {
            return None;
        }
        let unit = (adcn - 1) as u8;
        let (_, calib_version) = Self::block_version();
        let version_number = calib_version;

        match version_number {
            1 => {
                let low = Self::rtc_table_get_parsed_efuse_value(Self::rtc_table_get_tag(
                    version_number,
                    unit,
                    atten,
                    RtcCalibParam::V1VLow,
                )?);
                let high = Self::rtc_table_get_parsed_efuse_value(Self::rtc_table_get_tag(
                    version_number,
                    unit,
                    atten,
                    RtcCalibParam::V1VHigh,
                )?);

                let vhigh = RTC_CALIB_V_HIGH[atten as usize];
                let vlow = RTC_CALIB_V_LOW;
                let dx = high - low;
                Some((vlow * high - vhigh * low) / dx)
            }
            2 => Some(0),
            _ => None,
        }
    }
    /// The value to pass to set_init_code. Only present in version 2 of the
    /// efuse.
    pub fn rtc_calib_init_code(adcn: usize, atten: Attenuation) -> Option<u16> {
        if adcn > 2 || adcn == 0 {
            return None;
        }
        let unit = (adcn - 1) as u8;
        let (_, calib_version) = Self::block_version();
        let version_number = calib_version;

        if version_number == 2 {
            let icode = Self::rtc_table_get_parsed_efuse_value(Self::rtc_table_get_tag(
                version_number,
                unit,
                atten,
                RtcCalibParam::V2VInit,
            )?);
            Some(icode as u16)
        } else {
            None
        }
    }

    /// Return the calibration voltage in mV. It's only used as dy in the
    /// slope calculation of the AdcCalLine::new_cal calibration. Correct it
    /// to be ready for this calculation.
    pub fn rtc_calib_cal_mv(adcn: usize, atten: Attenuation) -> u16 {
        if adcn > 2 || adcn == 0 {
            return 0;
        }
        let _unit = (adcn - 1) as u8;

        let index = atten as usize;
        if index >= RTC_CALIB_V_HIGH.len() {
            return 0;
        }

        let (_, calib_version) = Self::block_version();
        let version_number = calib_version;

        // Note: ... - ..._V_LOW because it's used as dy to calculate the slope in
        // AdcCalLine::new_cal
        (match version_number {
            1 => RTC_CALIB_V_HIGH[index] - RTC_CALIB_V_LOW,
            _ => RTC_CALIB_V_HIGH[index],
        }) as u16
    }

    /// Return the calibration adc value. It's only used as dx in the
    /// slope calculation of the AdcCalLine::new_cal calibration. Correct it
    /// to be ready for this calculation.
    pub fn rtc_calib_cal_code(adcn: usize, atten: Attenuation) -> Option<u16> {
        if adcn > 2 || adcn == 0 {
            return None;
        }
        let unit = (adcn - 1) as u8;
        let (_, calib_version) = Self::block_version();
        let version_number = calib_version;

        match version_number {
            1 => {
                let low = Self::rtc_table_get_parsed_efuse_value(Self::rtc_table_get_tag(
                    version_number,
                    unit,
                    atten,
                    RtcCalibParam::V1VLow,
                )?);
                let high = Self::rtc_table_get_parsed_efuse_value(Self::rtc_table_get_tag(
                    version_number,
                    unit,
                    atten,
                    RtcCalibParam::V1VHigh,
                )?);
                // Note: Not just high, but dx, because its purpose is in AdcCalLine::new_cal to
                // compute the slope
                let dx = high - low;
                Some(dx as u16)
            }
            2 => {
                let high = Self::rtc_table_get_parsed_efuse_value(Self::rtc_table_get_tag(
                    version_number,
                    unit,
                    atten,
                    RtcCalibParam::V2VHigh,
                )?);
                Some(high as u16)
            }
            _ => None,
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) enum EfuseBlock {
    Block0,
    Block1,
    Block2,
    Block3,
    Block4,
    Block5,
    Block6,
    Block7,
    Block8,
    Block9,
    Block10,
}

impl EfuseBlock {
    pub(crate) fn address(self) -> *const u32 {
        let efuse = EFUSE::regs();
        match self {
            Self::Block0 => efuse.rd_wr_dis().as_ptr(),
            Self::Block1 => efuse.rd_mac_spi_sys_0().as_ptr(),
            Self::Block2 => efuse.rd_sys_data_part1_(0).as_ptr(),
            Self::Block3 => efuse.rd_usr_data(0).as_ptr(),
            Self::Block4 => efuse.rd_key0_data(0).as_ptr(),
            Self::Block5 => efuse.rd_key1_data(0).as_ptr(),
            Self::Block6 => efuse.rd_key2_data(0).as_ptr(),
            Self::Block7 => efuse.rd_key3_data(0).as_ptr(),
            Self::Block8 => efuse.rd_key4_data(0).as_ptr(),
            Self::Block9 => efuse.rd_key5_data(0).as_ptr(),
            Self::Block10 => efuse.rd_sys_data_part2_(0).as_ptr(),
        }
    }
}

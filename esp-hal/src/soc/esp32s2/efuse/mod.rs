#![cfg_attr(docsrs, procmacros::doc_replace)]
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
//! # {before_snippet}
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
//! # {after_snippet}
//! ```

pub use self::fields::*;
use crate::{analog::adc::Attenuation, peripherals::EFUSE, soc::efuse_field::EfuseField};

mod fields;

/// A struct representing the eFuse functionality of the chip.
pub struct Efuse;

pub enum RtcCalibParam {
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

// https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/efuse/esp32s2/include/esp_efuse_rtc_table.h#L31C1-L65C34
// These are the tags. Either use them directly or use
// esp_efuse_rtc_table_get_tag to calculate the corresponding tag.
const RTCCALIB_V1IDX_A10L: usize = 1;
const RTCCALIB_V1IDX_A11L: usize = 2;
const RTCCALIB_V1IDX_A12L: usize = 3;
const RTCCALIB_V1IDX_A13L: usize = 4;
const RTCCALIB_V1IDX_A20L: usize = 5;
const RTCCALIB_V1IDX_A21L: usize = 6;
const RTCCALIB_V1IDX_A22L: usize = 7;
const RTCCALIB_V1IDX_A23L: usize = 8;
const RTCCALIB_V1IDX_A10H: usize = 9;
const RTCCALIB_V1IDX_A11H: usize = 10;
const RTCCALIB_V1IDX_A12H: usize = 11;
const RTCCALIB_V1IDX_A13H: usize = 12;
const RTCCALIB_V1IDX_A20H: usize = 13;
const RTCCALIB_V1IDX_A21H: usize = 14;
const RTCCALIB_V1IDX_A22H: usize = 15;
const RTCCALIB_V1IDX_A23H: usize = 16;
const RTCCALIB_V2IDX_A10H: usize = 17;
const RTCCALIB_V2IDX_A11H: usize = 18;
const RTCCALIB_V2IDX_A12H: usize = 19;
const RTCCALIB_V2IDX_A13H: usize = 20;
const RTCCALIB_V2IDX_A20H: usize = 21;
const RTCCALIB_V2IDX_A21H: usize = 22;
const RTCCALIB_V2IDX_A22H: usize = 23;
const RTCCALIB_V2IDX_A23H: usize = 24;
const RTCCALIB_V2IDX_A10I: usize = 25;
const RTCCALIB_V2IDX_A11I: usize = 26;
const RTCCALIB_V2IDX_A12I: usize = 27;
const RTCCALIB_V2IDX_A13I: usize = 28;
const RTCCALIB_V2IDX_A20I: usize = 29;
const RTCCALIB_V2IDX_A21I: usize = 30;
const RTCCALIB_V2IDX_A22I: usize = 31;
const RTCCALIB_V2IDX_A23I: usize = 32;
const RTCCALIB_IDX_TMPSENSOR: usize = 33;

impl Efuse {
    /// Get status of SPI boot encryption.
    pub fn flash_encryption() -> bool {
        !Self::read_field_le::<u8>(SPI_BOOT_CRYPT_CNT)
            .count_ones()
            .is_multiple_of(2)
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
            // TODO: Check BE/LE
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
    pub fn rtc_table_get_tag(
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
            (1, V1VLow) => RTCCALIB_V1IDX_A10L,
            (1, V1VHigh) => RTCCALIB_V1IDX_A10H,
            (2, V2VInit) => RTCCALIB_V2IDX_A10I,
            (2, V2VHigh) => RTCCALIB_V2IDX_A10H,
            _ => return None,
        };

        const RTCCALIB_ATTENCOUNT: usize = 4;
        Some(param_offset + (unit as usize) * RTCCALIB_ATTENCOUNT + (atten as usize))
    }

    pub fn rtc_table_get_raw_efuse_value(tag: usize) -> i32 {
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
        // TODO: Check BE/LE
        let val = Self::read_field_be(EfuseField::new(info.block, info.begin_bit, info.length));
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
}

#[derive(Debug, Clone, Copy, strum::FromRepr)]
#[repr(u32)]
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

/// See <https://github.com/espressif/esp-idf/blob/a45d713b03fd96d8805d1cc116f02a4415b360c7/components/efuse/esp32s2/esp_efuse_rtc_table.c#L48>
struct MapInfo {
    block: EfuseBlock,
    begin_bit: u16,
    length: u16,
    multiplier: i32,
    base: i32,
    dependency: usize,
}

const RAW_MAP: [MapInfo; 34] = [
    MapInfo {
        block: EfuseBlock::Block0,
        begin_bit: 0,
        length: 0,
        multiplier: 0,
        base: 0,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 208,
        length: 6,
        multiplier: 4,
        base: 2231,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 214,
        length: 6,
        multiplier: 4,
        base: 1643,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 220,
        length: 6,
        multiplier: 4,
        base: 1290,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 226,
        length: 6,
        multiplier: 4,
        base: 701,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 232,
        length: 6,
        multiplier: 4,
        base: 2305,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 238,
        length: 6,
        multiplier: 4,
        base: 1693,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 244,
        length: 6,
        multiplier: 4,
        base: 1343,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 250,
        length: 6,
        multiplier: 4,
        base: 723,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 144,
        length: 8,
        multiplier: 4,
        base: 5775,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 152,
        length: 8,
        multiplier: 4,
        base: 5693,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 160,
        length: 8,
        multiplier: 4,
        base: 5723,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 168,
        length: 8,
        multiplier: 4,
        base: 6209,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 176,
        length: 8,
        multiplier: 4,
        base: 5817,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 184,
        length: 8,
        multiplier: 4,
        base: 5703,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 192,
        length: 8,
        multiplier: 4,
        base: 5731,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 200,
        length: 8,
        multiplier: 4,
        base: 6157,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 197,
        length: 6,
        multiplier: 2,
        base: 169,
        dependency: RTCCALIB_V2IDX_A12H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 203,
        length: 6,
        multiplier: 2,
        base: -26,
        dependency: RTCCALIB_V2IDX_A12H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 209,
        length: 9,
        multiplier: 2,
        base: 126,
        dependency: RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 218,
        length: 7,
        multiplier: 2,
        base: 387,
        dependency: RTCCALIB_V2IDX_A12H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 225,
        length: 7,
        multiplier: 2,
        base: 177,
        dependency: RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 232,
        length: 10,
        multiplier: 2,
        base: 5815,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 242,
        length: 7,
        multiplier: 2,
        base: 27,
        dependency: RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 249,
        length: 7,
        multiplier: 2,
        base: 410,
        dependency: RTCCALIB_V2IDX_A21H,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 147,
        length: 8,
        multiplier: 2,
        base: 1519,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 155,
        length: 6,
        multiplier: 2,
        base: 88,
        dependency: RTCCALIB_V2IDX_A10I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 161,
        length: 5,
        multiplier: 2,
        base: 8,
        dependency: RTCCALIB_V2IDX_A11I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 166,
        length: 6,
        multiplier: 2,
        base: 70,
        dependency: RTCCALIB_V2IDX_A12I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 172,
        length: 8,
        multiplier: 2,
        base: 1677,
        dependency: 0,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 180,
        length: 6,
        multiplier: 2,
        base: 23,
        dependency: RTCCALIB_V2IDX_A20I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 186,
        length: 5,
        multiplier: 2,
        base: 6,
        dependency: RTCCALIB_V2IDX_A21I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 191,
        length: 6,
        multiplier: 2,
        base: 13,
        dependency: RTCCALIB_V2IDX_A22I,
    },
    MapInfo {
        block: EfuseBlock::Block2,
        begin_bit: 135,
        length: 9,
        multiplier: 1,
        base: 0,
        dependency: 0,
    },
];

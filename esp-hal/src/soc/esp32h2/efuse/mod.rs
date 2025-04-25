//! # Reading of eFuses (ESP32-H2)
//!
//! ## Overview
//!
//! The `efuse` module provides functionality for reading eFuse data
//! from the `ESP32-H2` chip, allowing access to various chip-specific
//! information such as:
//!
//!   * MAC address
//!   * ADC calibration data
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

pub use self::fields::*;
use crate::{analog::adc::Attenuation, peripherals::EFUSE};

mod fields;

/// A struct representing the eFuse functionality of the chip.
pub struct Efuse;

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
    /// See: <https://github.com/espressif/esp-idf/blob/dc016f5987/components/hal/efuse_hal.c#L27-L30>
    pub fn block_version() -> (u8, u8) {
        (
            Self::read_field_le::<u8>(BLK_VERSION_MAJOR),
            Self::read_field_le::<u8>(BLK_VERSION_MINOR),
        )
    }

    /// Get version of RTC calibration block
    ///
    /// See: <https://github.com/espressif/esp-idf/blob/be06a6f/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L20>
    /// //esp_efuse_rtc_calib_get_ver
    pub fn rtc_calib_version() -> u8 {
        let (_major, minor) = Self::block_version();
        if minor >= 2 { 1 } else { 0 }
    }

    /// Get ADC initial code for specified attenuation from efuse
    ///
    /// See: <https://github.com/espressif/esp-idf/blob/be06a6f/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L33>
    pub fn rtc_calib_init_code(_unit: u8, atten: Attenuation) -> Option<u16> {
        let version = Self::rtc_calib_version();

        if version > 4 {
            return None;
        }

        // See: <https://github.com/espressif/esp-idf/blob/be06a6f/components/efuse/esp32h2/esp_efuse_table.csv#L76-L79>
        let init_code: u16 = Self::read_field_le(match atten {
            Attenuation::_0dB => ADC1_AVE_INITCODE_ATTEN0,
            Attenuation::_2p5dB => ADC1_AVE_INITCODE_ATTEN1,
            Attenuation::_6dB => ADC1_AVE_INITCODE_ATTEN2,
            Attenuation::_11dB => ADC1_AVE_INITCODE_ATTEN3,
        });

        Some(init_code + 1600) // version 1 logic
    }

    /// Get ADC reference point voltage for specified attenuation in millivolts
    ///
    /// See: <https://github.com/espressif/esp-idf/blob/be06a6f/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L91>
    pub fn rtc_calib_cal_mv(_unit: u8, atten: Attenuation) -> u16 {
        const INPUT_VOUT_MV: [[u16; 4]; 1] = [
            [750, 1000, 1500, 2800], // Calibration V1 coefficients
        ];

        let version = Self::rtc_calib_version();

        // https://github.com/espressif/esp-idf/blob/master/components/efuse/esp32h2/include/esp_efuse_rtc_calib.h#L15C9-L17
        // ESP_EFUSE_ADC_CALIB_VER1     1
        // ESP_EFUSE_ADC_CALIB_VER_MIN  ESP_EFUSE_ADC_CALIB_VER1
        // ESP_EFUSE_ADC_CALIB_VER_MAX  ESP_EFUSE_ADC_CALIB_VER1
        if version != 1 {
            // The required efuse bits for this chip are not burnt.
            // 1100 is the middle of the reference voltage range.
            // See: <https://github.com/espressif/esp-idf/blob/465b159cd8771ffab6be70c7675ecf6705b62649/docs/en/api-reference/peripherals/adc_calibration.rst?plain=1#L9>
            return 1100;
        }

        INPUT_VOUT_MV[version as usize - 1][atten as usize]
    }

    /// Returns the call code
    ///
    /// See: <https://github.com/espressif/esp-idf/blob/17a2461297076481858b7f76482676a521cc727a/components/efuse/esp32h2/esp_efuse_rtc_calib.c#L91>
    pub fn rtc_calib_cal_code(_unit: u8, atten: Attenuation) -> Option<u16> {
        let cal_code: u16 = Self::read_field_le(match atten {
            Attenuation::_0dB => ADC1_HI_DOUT_ATTEN0,
            Attenuation::_2p5dB => ADC1_HI_DOUT_ATTEN1,
            Attenuation::_6dB => ADC1_HI_DOUT_ATTEN2,
            Attenuation::_11dB => ADC1_HI_DOUT_ATTEN3,
        });
        let cal_code: u16 = if atten == Attenuation::_6dB {
            2970 + cal_code
        } else {
            2900 + cal_code
        };
        Some(cal_code)
    }

    /// Returns the major hardware revision
    pub fn major_chip_version() -> u8 {
        Self::read_field_le(WAFER_VERSION_MAJOR)
    }

    /// Returns the minor hardware revision
    pub fn minor_chip_version() -> u8 {
        Self::read_field_le(WAFER_VERSION_MINOR)
    }

    /// Returns the hardware revision
    ///
    /// The chip version is calculated using the following
    /// formula: MAJOR * 100 + MINOR. (if the result is 1, then version is v0.1)
    pub fn chip_revision() -> u16 {
        Self::major_chip_version() as u16 * 100 + Self::minor_chip_version() as u16
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
            Self::Block1 => efuse.rd_mac_sys_0().as_ptr(),
            Self::Block2 => efuse.rd_sys_part1_data0().as_ptr(),
            Self::Block3 => efuse.rd_usr_data0().as_ptr(),
            Self::Block4 => efuse.rd_key0_data0().as_ptr(),
            Self::Block5 => efuse.rd_key1_data0().as_ptr(),
            Self::Block6 => efuse.rd_key2_data0().as_ptr(),
            Self::Block7 => efuse.rd_key3_data0().as_ptr(),
            Self::Block8 => efuse.rd_key4_data0().as_ptr(),
            Self::Block9 => efuse.rd_key5_data0().as_ptr(),
            Self::Block10 => efuse.rd_sys_part2_data0().as_ptr(),
        }
    }
}

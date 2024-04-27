//! # Reading of eFuses (ESP32-C3)
//!
//! ## Overview
//!
//! The `efuse` module provides functionality for reading eFuse data
//! from the `ESP32-C3` chip, allowing access to various chip-specific
//! information such as :
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
//! ## Example
//!
//! ### Read chip's MAC address from the eFuse storage.
//! ```no_run
//! let mac_address = Efuse::read_base_mac_address();
//! writeln!(
//!     serial_tx,
//!     "MAC: {:#X}:{:#X}:{:#X}:{:#X}:{:#X}:{:#X}",
//!     mac_address[0],
//!     mac_address[1],
//!     mac_address[2],
//!     mac_address[3],
//!     mac_address[4],
//!     mac_address[5]
//! );
//! ```

pub use self::fields::*;
use crate::{analog::adc::Attenuation, peripherals::EFUSE};

mod fields;

pub struct Efuse;

impl Efuse {
    /// Reads chip's MAC address from the eFuse storage.
    pub fn read_base_mac_address() -> [u8; 6] {
        Self::read_field_be(MAC)
    }

    /// Get status of SPI boot encryption.
    pub fn get_flash_encryption() -> bool {
        (Self::read_field_le::<u8>(SPI_BOOT_CRYPT_CNT).count_ones() % 2) != 0
    }

    /// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
    pub fn get_rwdt_multiplier() -> u8 {
        Self::read_field_le::<u8>(WDT_DELAY_SEL)
    }

    /// Get efuse block version
    ///
    /// see <https://github.com/espressif/esp-idf/blob/dc016f5987/components/hal/efuse_hal.c#L27-L30>
    pub fn get_block_version() -> (u8, u8) {
        // see <https://github.com/espressif/esp-idf/blob/dc016f5987/components/hal/esp32c3/include/hal/efuse_ll.h#L70-L78>
        // <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_table.csv#L163>
        // <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_table.csv#L173>
        (
            Self::read_field_le::<u8>(BLK_VERSION_MAJOR),
            Self::read_field_le::<u8>(BLK_VERSION_MINOR),
        )
    }

    /// Get version of RTC calibration block
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_rtc_calib.c#L12>
    pub fn get_rtc_calib_version() -> u8 {
        let (major, _minor) = Self::get_block_version();
        if major == 1 {
            1
        } else {
            0
        }
    }

    /// Get ADC initial code for specified attenuation from efuse
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_rtc_calib.c#L25>
    pub fn get_rtc_calib_init_code(_unit: u8, atten: Attenuation) -> Option<u16> {
        let version = Self::get_rtc_calib_version();

        if version != 1 {
            return None;
        }

        // See <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_table.csv#L176-L179>
        let init_code: u16 = Self::read_field_le(match atten {
            Attenuation::Attenuation0dB => ADC1_INIT_CODE_ATTEN0,
            Attenuation::Attenuation2p5dB => ADC1_INIT_CODE_ATTEN1,
            Attenuation::Attenuation6dB => ADC1_INIT_CODE_ATTEN2,
            Attenuation::Attenuation11dB => ADC1_INIT_CODE_ATTEN3,
        });

        Some(init_code + 1000) // version 1 logic
    }

    /// Get ADC reference point voltage for specified attenuation in millivolts
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_rtc_calib.c#L49>
    pub fn get_rtc_calib_cal_mv(_unit: u8, atten: Attenuation) -> u16 {
        match atten {
            Attenuation::Attenuation0dB => 400,
            Attenuation::Attenuation2p5dB => 550,
            Attenuation::Attenuation6dB => 750,
            Attenuation::Attenuation11dB => 1370,
        }
    }

    /// Get ADC reference point digital code for specified attenuation
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_rtc_calib.c#L49>
    pub fn get_rtc_calib_cal_code(_unit: u8, atten: Attenuation) -> Option<u16> {
        let version = Self::get_rtc_calib_version();

        if version != 1 {
            return None;
        }

        // See <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c3/esp_efuse_table.csv#L180-L183>
        let cal_code: u16 = Self::read_field_le(match atten {
            Attenuation::Attenuation0dB => ADC1_CAL_VOL_ATTEN0,
            Attenuation::Attenuation2p5dB => ADC1_CAL_VOL_ATTEN1,
            Attenuation::Attenuation6dB => ADC1_CAL_VOL_ATTEN2,
            Attenuation::Attenuation11dB => ADC1_CAL_VOL_ATTEN3,
        });

        let cal_code = if cal_code & (1 << 9) != 0 {
            2000 - (cal_code & !(1 << 9))
        } else {
            2000 + cal_code
        };

        Some(cal_code)
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
        use EfuseBlock::*;
        let efuse = unsafe { &*EFUSE::ptr() };
        match self {
            Block0 => efuse.rd_wr_dis().as_ptr(),
            Block1 => efuse.rd_mac_spi_sys_0().as_ptr(),
            Block2 => efuse.rd_sys_part1_data0().as_ptr(),
            Block3 => efuse.rd_usr_data0().as_ptr(),
            Block4 => efuse.rd_key0_data0().as_ptr(),
            Block5 => efuse.rd_key1_data0().as_ptr(),
            Block6 => efuse.rd_key2_data0().as_ptr(),
            Block7 => efuse.rd_key3_data0().as_ptr(),
            Block8 => efuse.rd_key4_data0().as_ptr(),
            Block9 => efuse.rd_key5_data0().as_ptr(),
            Block10 => efuse.rd_sys_part2_data0().as_ptr(),
        }
    }
}

//! # Reading of eFuses (ESP32-C2)
//!
//! ## Overview
//! The `efuse` module provides functionality for reading eFuse data
//! from the `ESP32-C2` chip, allowing access to various chip-specific
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
//! ## Examples
//! ### Read chip's MAC address from the eFuse storage.
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::efuse::Efuse;
//! # use esp_hal::gpio::Io;
//! # use esp_hal::uart::Uart;
//! # use core::writeln;
//! # use core::fmt::Write;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let mut serial_tx = Uart::new(peripherals.UART0, &clocks, io.pins.gpio4, io.pins.gpio5).unwrap();
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
        // see <https://github.com/espressif/esp-idf/blob/dc016f5987/components/hal/esp32c2/include/hal/efuse_ll.h#L65-L73>
        // <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_table.csv#L90-L91>
        (
            Self::read_field_le::<u8>(BLK_VERSION_MAJOR),
            Self::read_field_le::<u8>(BLK_VERSION_MINOR),
        )
    }

    /// Get version of RTC calibration block
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_rtc_calib.c#L14>
    pub fn get_rtc_calib_version() -> u8 {
        let (major, _minor) = Self::get_block_version();
        if major == 0 {
            1
        } else {
            0
        }
    }

    /// Get ADC initial code for specified attenuation from efuse
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_rtc_calib.c#L27>
    pub fn get_rtc_calib_init_code(_unit: u8, atten: Attenuation) -> Option<u16> {
        let version = Self::get_rtc_calib_version();

        if version != 1 {
            return None;
        }

        // see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_table.csv#L94>
        let diff_code0: u16 = Self::read_field_le(ADC1_INIT_CODE_ATTEN0);
        let code0 = if diff_code0 & (1 << 7) != 0 {
            2160 - (diff_code0 & 0x7f)
        } else {
            2160 + diff_code0
        };

        if matches!(atten, Attenuation::Attenuation0dB) {
            return Some(code0);
        }

        // see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_table.csv#L95>
        let diff_code11: u16 = Self::read_field_le(ADC1_INIT_CODE_ATTEN3);
        let code11 = code0 + diff_code11;

        Some(code11)
    }

    /// Get ADC reference point voltage for specified attenuation in millivolts
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_rtc_calib.c#L65>
    pub fn get_rtc_calib_cal_mv(_unit: u8, atten: Attenuation) -> u16 {
        match atten {
            Attenuation::Attenuation0dB => 400,
            Attenuation::Attenuation11dB => 1370,
        }
    }

    /// Get ADC reference point digital code for specified attenuation
    ///
    /// see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_rtc_calib.c#L65>
    pub fn get_rtc_calib_cal_code(_unit: u8, atten: Attenuation) -> Option<u16> {
        let version = Self::get_rtc_calib_version();

        if version != 1 {
            return None;
        }

        // see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_table.csv#L96>
        let diff_code0: u16 = Self::read_field_le(ADC1_CAL_VOL_ATTEN0);
        let code0 = if diff_code0 & (1 << 7) != 0 {
            1540 - (diff_code0 & 0x7f)
        } else {
            1540 + diff_code0
        };

        if matches!(atten, Attenuation::Attenuation0dB) {
            return Some(code0);
        }

        // see <https://github.com/espressif/esp-idf/blob/903af13e8/components/efuse/esp32c2/esp_efuse_table.csv#L97>
        let diff_code11: u16 = Self::read_field_le(ADC1_CAL_VOL_ATTEN3);
        let code11 = if diff_code0 & (1 << 5) != 0 {
            code0 - (diff_code11 & 0x1f)
        } else {
            code0 + diff_code11
        } - 123;

        Some(code11)
    }
}

#[derive(Copy, Clone)]
pub(crate) enum EfuseBlock {
    Block0,
    Block1,
    Block2,
    Block3,
}

impl EfuseBlock {
    pub(crate) fn address(self) -> *const u32 {
        use EfuseBlock::*;
        let efuse = unsafe { &*EFUSE::ptr() };
        match self {
            Block0 => efuse.rd_wr_dis().as_ptr(),
            Block1 => efuse.rd_blk1_data0().as_ptr(),
            Block2 => efuse.rd_blk2_data0().as_ptr(),
            Block3 => efuse.rd_blk3_data0().as_ptr(),
        }
    }
}

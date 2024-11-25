//! # Reading of eFuses (ESP32)
//!
//! ## Overview
//!
//! The `efuse` module provides functionality for reading eFuse data
//! from the `ESP32` chip, allowing access to various chip-specific information
//! such as:
//!
//!   * MAC address
//!   * Chip type, revision
//!   * Core count
//!   * Max CPU frequency
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
//! println!("Core Count {}", Efuse::core_count());
//! println!("Bluetooth enabled {}", Efuse::is_bluetooth_enabled());
//! println!("Chip type {:?}", Efuse::chip_type());
//! println!("Max CPU clock {:?}", Efuse::max_cpu_frequency());
//! # }
//! ```

use fugit::{HertzU32, RateExtU32};

pub use self::fields::*;
use crate::peripherals::EFUSE;

mod fields;

/// A struct representing the eFuse functionality of the chip.
pub struct Efuse;

/// Representing different types of ESP32 chips.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum ChipType {
    /// Represents the ESP32 D0WDQ6 chip variant.
    Esp32D0wdq6,
    /// Represents the ESP32 D0WDQ5 chip variant.
    Esp32D0wdq5,
    /// Represents the ESP32 D2WDQ5 chip variant.
    Esp32D2wdq5,
    /// Represents the ESP32 Pico D2 chip variant.
    Esp32Picod2,
    /// Represents the ESP32 Pico D4 chip variant.
    Esp32Picod4,
    /// Represents the ESP32 Pico v3.02 chip variant.
    Esp32Picov302,
    /// Represents an unknown or unsupported chip variant.
    Unknown,
}

impl Efuse {
    /// Reads the base MAC address from the eFuse memory.
    pub fn read_base_mac_address() -> [u8; 6] {
        Self::read_field_be(MAC)
    }

    /// Returns the number of CPUs available on the chip.
    ///
    /// While ESP32 chips usually come with two mostly equivalent CPUs (protocol
    /// CPU and application CPU), the application CPU is unavailable on
    /// some.
    pub fn core_count() -> u32 {
        if Self::read_bit(DISABLE_APP_CPU) {
            1
        } else {
            2
        }
    }

    /// Returns the maximum rated clock of the CPU in MHz.
    ///
    /// Note that the actual clock may be lower, depending on the current power
    /// configuration of the chip, clock source, and other settings.
    pub fn max_cpu_frequency() -> HertzU32 {
        let has_rating = Self::read_bit(CHIP_CPU_FREQ_RATED);
        let has_low_rating = Self::read_bit(CHIP_CPU_FREQ_LOW);

        if has_rating && has_low_rating {
            160.MHz()
        } else {
            240.MHz()
        }
    }

    /// Returns the CHIP_VER_DIS_BT eFuse value.
    pub fn is_bluetooth_enabled() -> bool {
        !Self::read_bit(DISABLE_BT)
    }

    /// Returns the CHIP_VER_PKG eFuse value.
    pub fn chip_type() -> ChipType {
        let chip_ver = Self::read_field_le::<u8>(CHIP_PACKAGE)
            | Self::read_field_le::<u8>(CHIP_PACKAGE_4BIT) << 4;

        match chip_ver {
            0 => ChipType::Esp32D0wdq6,
            1 => ChipType::Esp32D0wdq5,
            2 => ChipType::Esp32D2wdq5,
            4 => ChipType::Esp32Picod2,
            5 => ChipType::Esp32Picod4,
            6 => ChipType::Esp32Picov302,
            _ => ChipType::Unknown,
        }
    }

    /// Get status of SPI boot encryption.
    pub fn flash_encryption() -> bool {
        (Self::read_field_le::<u8>(FLASH_CRYPT_CNT).count_ones() % 2) != 0
    }
}

#[allow(unused)]
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
            Block0 => efuse.blk0_rdata0().as_ptr(),
            Block1 => efuse.blk1_rdata0().as_ptr(),
            Block2 => efuse.blk2_rdata0().as_ptr(),
            Block3 => efuse.blk3_rdata0().as_ptr(),
        }
    }
}

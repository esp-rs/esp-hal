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
//! ## Example
//!
//! ### Read chip's MAC address from the eFuse storage.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::efuse::Efuse;
//! # use esp_hal::gpio::Io;
//! # use esp_hal::uart::Uart;
//! # use core::writeln;
//! # use core::fmt::Write;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let mut serial_tx = Uart::new(peripherals.UART0, io.pins.gpio4, io.pins.gpio5).unwrap();
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
use crate::peripherals::EFUSE;

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
            Block1 => efuse.rd_mac_sys_0().as_ptr(),
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

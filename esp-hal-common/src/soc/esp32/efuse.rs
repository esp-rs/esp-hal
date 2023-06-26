//! Reading of eFuses

use fugit::{HertzU32, RateExtU32};

use crate::peripherals::EFUSE;
pub use crate::soc::efuse_field::*;

pub struct Efuse;

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum ChipType {
    Esp32D0wdq6,
    Esp32D0wdq5,
    Esp32D2wdq5,
    Esp32Picod2,
    Esp32Picod4,
    Unknown,
}

impl Efuse {
    /// Reads chip's MAC address from the eFuse storage.
    ///
    /// # Example
    ///
    /// ```
    /// let mac_address = Efuse::get_mac_address();
    /// writeln!(
    ///     serial_tx,
    ///     "MAC: {:#X}:{:#X}:{:#X}:{:#X}:{:#X}:{:#X}",
    ///     mac_address[0],
    ///     mac_address[1],
    ///     mac_address[2],
    ///     mac_address[3],
    ///     mac_address[4],
    ///     mac_address[5]
    /// );
    /// ```
    pub fn get_mac_address() -> [u8; 6] {
        Self::read_field_be(MAC_FACTORY)
    }

    /// Returns the number of CPUs available on the chip.
    ///
    /// While ESP32 chips usually come with two mostly equivalent CPUs (protocol
    /// CPU and application CPU), the application CPU is unavailable on
    /// some.
    pub fn get_core_count() -> u32 {
        if Self::read_field_le::<bool>(DISABLE_APP_CPU) {
            1
        } else {
            2
        }
    }

    /// Returns the maximum rated clock of the CPU in MHz.
    ///
    /// Note that the actual clock may be lower, depending on the current power
    /// configuration of the chip, clock source, and other settings.
    pub fn get_max_cpu_frequency() -> HertzU32 {
        let has_rating = Self::read_field_le::<bool>(CHIP_CPU_FREQ_RATED);
        let has_low_rating = Self::read_field_le::<bool>(CHIP_CPU_FREQ_LOW);

        if has_rating && has_low_rating {
            160u32.MHz()
        } else {
            240u32.MHz()
        }
    }

    /// Returns the CHIP_VER_DIS_BT eFuse value.
    pub fn is_bluetooth_enabled() -> bool {
        !Self::read_field_le::<bool>(DISABLE_BT)
    }

    /// Returns the CHIP_VER_PKG eFuse value.
    pub fn get_chip_type() -> ChipType {
        let chip_ver = Self::read_field_le::<u8>(CHIP_PACKAGE)
            | Self::read_field_le::<u8>(CHIP_PACKAGE_4BIT) << 4;

        match chip_ver {
            0 => ChipType::Esp32D0wdq6,
            1 => ChipType::Esp32D0wdq5,
            2 => ChipType::Esp32D2wdq5,
            4 => ChipType::Esp32Picod2,
            5 => ChipType::Esp32Picod4,
            _ => ChipType::Unknown,
        }
    }

    /// Get status of SPI boot encryption.
    pub fn get_flash_encryption() -> bool {
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
            Block0 => efuse.blk0_rdata0.as_ptr(),
            Block1 => efuse.blk1_rdata0.as_ptr(),
            Block2 => efuse.blk2_rdata0.as_ptr(),
            Block3 => efuse.blk3_rdata0.as_ptr(),
        }
    }
}

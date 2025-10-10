use crate::{peripherals::EFUSE, time::Rate};

mod fields;
pub use fields::*;

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

impl super::Efuse {
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
    pub fn max_cpu_frequency() -> Rate {
        let has_rating = Self::read_bit(CHIP_CPU_FREQ_RATED);
        let has_low_rating = Self::read_bit(CHIP_CPU_FREQ_LOW);

        if has_rating && has_low_rating {
            Rate::from_mhz(160)
        } else {
            Rate::from_mhz(240)
        }
    }

    /// Returns the CHIP_VER_DIS_BT eFuse value.
    pub fn is_bluetooth_enabled() -> bool {
        !Self::read_bit(DISABLE_BT)
    }

    /// Returns the CHIP_VER_PKG eFuse value.
    pub fn chip_type() -> ChipType {
        let chip_ver = Self::read_field_le::<u8>(CHIP_PACKAGE)
            | (Self::read_field_le::<u8>(CHIP_PACKAGE_4BIT) << 4);

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

    /// Returns the major hardware revision
    pub fn major_chip_version() -> u8 {
        let eco_bit0 = Self::read_field_le::<u32>(CHIP_VER_REV1);
        let eco_bit1 = Self::read_field_le::<u32>(CHIP_VER_REV2);
        let eco_bit2 =
            (crate::peripherals::APB_CTRL::regs().date().read().bits() & 0x80000000) >> 31;

        match (eco_bit2 << 2) | (eco_bit1 << 1) | eco_bit0 {
            1 => 1,
            3 => 2,
            7 => 3,
            _ => 0,
        }
    }

    /// Returns the minor hardware revision
    pub fn minor_chip_version() -> u8 {
        Self::read_field_le(WAFER_VERSION_MINOR)
    }
}

#[derive(Debug, Clone, Copy, strum::FromRepr)]
#[repr(u32)]
pub(crate) enum EfuseBlock {
    Block0,
    Block1,
    Block2,
    Block3,
}

impl EfuseBlock {
    pub(crate) fn address(self) -> *const u32 {
        let efuse = EFUSE::regs();
        match self {
            Self::Block0 => efuse.blk0_rdata0().as_ptr(),
            Self::Block1 => efuse.blk1_rdata0().as_ptr(),
            Self::Block2 => efuse.blk2_rdata0().as_ptr(),
            Self::Block3 => efuse.blk3_rdata0().as_ptr(),
        }
    }
}

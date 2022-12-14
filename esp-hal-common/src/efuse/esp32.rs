//! Reading of eFuses

use fugit::{HertzU32, RateExtU32};

use crate::peripherals::EFUSE;

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
        let efuse = unsafe { &*EFUSE::ptr() };

        let mac_low: u32 = efuse.blk0_rdata1.read().rd_wifi_mac_crc_low().bits();
        let mac_high: u32 = efuse.blk0_rdata2.read().rd_wifi_mac_crc_high().bits();

        let mac_low_bytes = mac_low.to_be_bytes();
        let mac_high_bytes = mac_high.to_be_bytes();

        [
            mac_high_bytes[2],
            mac_high_bytes[3],
            mac_low_bytes[0],
            mac_low_bytes[1],
            mac_low_bytes[2],
            mac_low_bytes[3],
        ]
    }

    /// Returns the number of CPUs available on the chip.
    ///
    /// While ESP32 chips usually come with two mostly equivalent CPUs (protocol
    /// CPU and application CPU), the application CPU is unavailable on
    /// some.
    pub fn get_core_count() -> u32 {
        let efuse = unsafe { &*EFUSE::ptr() };

        let cpu_disabled = efuse.blk0_rdata3.read().rd_chip_ver_dis_app_cpu().bit();
        if cpu_disabled {
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
        let efuse = unsafe { &*EFUSE::ptr() };

        let has_rating = efuse.blk0_rdata3.read().rd_chip_cpu_freq_rated().bit();
        let has_low_rating = efuse.blk0_rdata3.read().rd_chip_cpu_freq_low().bit();

        if has_rating && has_low_rating {
            160u32.MHz()
        } else {
            240u32.MHz()
        }
    }

    /// Returns the CHIP_VER_DIS_BT eFuse value.
    pub fn is_bluetooth_enabled() -> bool {
        let efuse = unsafe { &*EFUSE::ptr() };

        !efuse.blk0_rdata3.read().rd_chip_ver_dis_bt().bit()
    }

    /// Returns the CHIP_VER_PKG eFuse value.
    pub fn get_chip_type() -> ChipType {
        let efuse = unsafe { &*EFUSE::ptr() };

        match efuse.blk0_rdata3.read().rd_chip_ver_pkg().bits() {
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
        let efuse = unsafe { &*EFUSE::ptr() };
        (efuse
            .blk0_rdata0
            .read()
            .rd_flash_crypt_cnt()
            .bits()
            .count_ones()
            % 2)
            != 0
    }
}

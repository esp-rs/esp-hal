//! Reading of eFuses

use crate::peripherals::EFUSE;

pub struct Efuse;

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

        let mac_low: u32 = efuse.rd_blk2_data0.read().bits();
        let mac_high: u16 = efuse.rd_blk2_data1.read().mac_id_high().bits();

        let mac_low_bytes = mac_low.to_be_bytes();
        let mac_high_bytes = mac_high.to_be_bytes();

        [
            mac_high_bytes[0],
            mac_high_bytes[1],
            mac_low_bytes[0],
            mac_low_bytes[1],
            mac_low_bytes[2],
            mac_low_bytes[3],
        ]
    }

    /// Get status of SPI boot encryption.
    pub fn get_flash_encryption() -> bool {
        let efuse = unsafe { &*EFUSE::ptr() };
        (efuse
            .rd_repeat_data0
            .read()
            .spi_boot_encrypt_decrypt_cnt()
            .bits()
            .count_ones()
            % 2)
            != 0
    }

    /// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
    pub fn get_rwdt_multiplier() -> u8 {
        let efuse = unsafe { &*EFUSE::ptr() };
        efuse.rd_repeat_data0.read().wdt_delay_sel().bits()
    }
}

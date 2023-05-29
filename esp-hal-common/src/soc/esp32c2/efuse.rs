//! Reading of eFuses

use crate::peripherals::EFUSE;
pub use crate::soc::efuse_field::*;

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
        Self::read_field_be(MAC_FACTORY)
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
}

impl EfuseBlock {
    pub(crate) fn address(self) -> *const u32 {
        use EfuseBlock::*;
        let efuse = unsafe { &*EFUSE::ptr() };
        match self {
            Block0 => efuse.rd_wr_dis.as_ptr(),
            Block1 => efuse.rd_blk1_data0.as_ptr(),
            Block2 => efuse.rd_blk2_data0.as_ptr(),
            Block3 => efuse.rd_blk3_data0.as_ptr(),
        }
    }
}

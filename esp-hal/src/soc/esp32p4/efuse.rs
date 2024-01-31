//! # Reading of eFuses (ESP32-P4)

use crate::peripherals::EFUSE;
pub use crate::soc::efuse_field::*;

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

#[derive(Clone, Copy)]
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
        let efuse = unsafe { &*EFUSE::PTR };
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

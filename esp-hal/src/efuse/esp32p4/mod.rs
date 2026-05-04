//! # Reading of eFuses (ESP32-P4)

use crate::peripherals::EFUSE;

#[cfg_attr(not(feature = "unstable"), allow(dead_code))]
mod fields;
#[instability::unstable]
pub use fields::*;

/// Get status of SPI boot encryption.
#[instability::unstable]
pub fn flash_encryption() -> bool {
    !super::read_field_le::<u8>(SPI_BOOT_CRYPT_CNT)
        .count_ones()
        .is_multiple_of(2)
}

/// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
#[instability::unstable]
pub fn rwdt_multiplier() -> u8 {
    super::read_field_le::<u8>(WDT_DELAY_SEL)
}

/// Returns the major hardware revision
pub(crate) fn major_chip_version() -> u8 {
    super::read_field_le(WAFER_VERSION_MAJOR)
}

/// Returns the minor hardware revision
pub(crate) fn minor_chip_version() -> u8 {
    super::read_field_le(WAFER_VERSION_MINOR)
}

#[derive(Debug, Clone, Copy, strum::FromRepr)]
#[repr(u32)]
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
        let efuse = EFUSE::regs();
        match self {
            Block0 => efuse.rd_wr_dis().as_ptr(),
            Block1 => efuse.rd_mac_sys0().as_ptr(),
            Block2 => efuse.rd_sys_part1_data(0).as_ptr(),
            Block3 => efuse.rd_usr_data(0).as_ptr(),
            Block4 => efuse.rd_key0_data(0).as_ptr(),
            Block5 => efuse.rd_key1_data(0).as_ptr(),
            Block6 => efuse.rd_key2_data(0).as_ptr(),
            Block7 => efuse.rd_key3_data(0).as_ptr(),
            Block8 => efuse.rd_key4_data(0).as_ptr(),
            Block9 => efuse.rd_key5_data(0).as_ptr(),
            Block10 => efuse.rd_sys_part2_data0().as_ptr(),
        }
    }
}

use crate::peripherals::EFUSE;

mod fields;
pub use fields::*;

impl super::Efuse {
    /// Get status of SPI boot encryption.
    pub fn flash_encryption() -> bool {
        !Self::read_field_le::<u8>(SPI_BOOT_CRYPT_CNT)
            .count_ones()
            .is_multiple_of(2)
    }

    /// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
    pub fn rwdt_multiplier() -> u8 {
        Self::read_field_le::<u8>(WDT_DELAY_SEL)
    }

    /// Returns the major hardware revision
    pub fn major_chip_version() -> u8 {
        Self::read_field_le(WAFER_VERSION_MAJOR)
    }

    /// Returns the minor hardware revision
    pub fn minor_chip_version() -> u8 {
        Self::read_field_le::<u8>(WAFER_VERSION_MINOR_HI) << 3
            | Self::read_field_le::<u8>(WAFER_VERSION_MINOR_LO)
    }
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
        let efuse = EFUSE::regs();
        match self {
            Self::Block0 => efuse.rd_wr_dis().as_ptr(),
            Self::Block1 => efuse.rd_mac_spi_sys_0().as_ptr(),
            Self::Block2 => efuse.rd_sys_data_part1_(0).as_ptr(),
            Self::Block3 => efuse.rd_usr_data(0).as_ptr(),
            Self::Block4 => efuse.rd_key0_data(0).as_ptr(),
            Self::Block5 => efuse.rd_key1_data(0).as_ptr(),
            Self::Block6 => efuse.rd_key2_data(0).as_ptr(),
            Self::Block7 => efuse.rd_key3_data(0).as_ptr(),
            Self::Block8 => efuse.rd_key4_data(0).as_ptr(),
            Self::Block9 => efuse.rd_key5_data(0).as_ptr(),
            Self::Block10 => efuse.rd_sys_data_part2_(0).as_ptr(),
        }
    }
}

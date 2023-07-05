//! ESP ROM libraries
//!
//! Safe abstractions to the additional libraries provided in the ESP's
//! read-only memory.

pub mod crc;
pub mod md5;

#[allow(unused)]
extern "C" {
    pub(crate) fn rom_i2c_writeReg(block: u32, block_hostid: u32, reg_add: u32, indata: u32);

    pub(crate) fn rom_i2c_writeReg_Mask(
        block: u32,
        block_hostid: u32,
        reg_add: u32,
        reg_add_msb: u32,
        reg_add_lsb: u32,
        indata: u32,
    );
}

#[doc(hidden)]
#[macro_export]
macro_rules! regi2c_write {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste::paste! {
            crate::rom::rom_i2c_writeReg(
                $block as _,
                [<$block _HOSTID>] as _,
                $reg_add as _,
                $indata as _
            );
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! regi2c_write_mask {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste::paste! {
            crate::rom::rom_i2c_writeReg_Mask(
                $block as _,
                [<$block _HOSTID>] as _,
                $reg_add as _,
                [<$reg_add _MSB>] as _,
                [<$reg_add _LSB>] as _,
                $indata as _
            );
        }
    };
}

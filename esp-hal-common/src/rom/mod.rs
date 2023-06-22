//! ESP ROM libraries
//!
//! Safe abstractions to the additional libraries provided in the ESP's
//! read-only memory.

pub use paste::paste;

pub mod crc;

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

#[macro_export]
macro_rules! regi2c_write {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste! {
            rom_i2c_writeReg($block,
                [<$block _HOSTID>],
                $reg_add,
                $indata
            );
        }
    };
}

#[macro_export]
macro_rules! regi2c_write_mask {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste! {
            rom_i2c_writeReg_Mask($block,
                [<$block _HOSTID>],
                $reg_add,
                [<$reg_add _MSB>],
                [<$reg_add _LSB>],
                $indata
            );
        }
    };
}

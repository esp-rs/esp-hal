pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod radio_clocks;

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

pub(crate) mod constants {
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 1;
}

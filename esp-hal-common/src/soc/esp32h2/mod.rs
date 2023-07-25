pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod radio_clocks;

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

pub(crate) mod constants {
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 2;

    pub const I2S_DEFAULT_CLK_SRC: u8 = 1;
    pub const I2S_SCLK: u32 = 96_000_000;

    pub const RMT_RAM_START: usize = 0x60007400;
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    pub const RMT_CLOCK_SRC: bool = false;
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(32);
}

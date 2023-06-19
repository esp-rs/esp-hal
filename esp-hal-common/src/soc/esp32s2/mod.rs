pub mod efuse;
pub mod gpio;
pub mod peripherals;
#[cfg(psram)]
pub mod psram;
pub mod radio_clocks;

pub(crate) mod constants {
    pub const I2S_SCLK: u32 = 160_000_000;
    pub const I2S_DEFAULT_CLK_SRC: u32 = 2;
}
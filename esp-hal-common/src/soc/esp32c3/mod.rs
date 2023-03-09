pub mod efuse;
pub mod gpio;
pub mod peripherals;

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

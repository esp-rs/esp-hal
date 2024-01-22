pub mod efuse;
pub mod gpio;
pub mod peripherals;

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x500D_6000;
}

pub(crate) mod constants {
    pub const SOC_DRAM_LOW: u32 = 0x4FF0_0000;
    pub const SOC_DRAM_HIGH: u32 = 0x4FFC_0000;
}

#[export_name = "__post_init"]
unsafe fn post_init() {
    // TODO: Disable watchdog timers
}

// pub mod efuse;
pub mod gpio;
// pub mod peripherals;

crate::unstable_module! {
    pub mod clocks;
}

pub(crate) mod regi2c;

pub(crate) use esp32p4 as pac;

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x500D_6000;
    pub const INTERRUPT_MAP_BASE_APP_CPU: u32 = 0x500D_6800;
}

pub(crate) fn pre_init() {
    // TODO: Check if anything needs to be done here
}

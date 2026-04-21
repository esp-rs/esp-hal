// pub mod efuse;
pub mod gpio;
// pub mod peripherals;

crate::unstable_module! {
    pub mod clocks;
}

pub(crate) mod regi2c;

pub(crate) use esp32p4 as pac;

// P4 DMA module alias: the esp-hal GDMA driver expects the GDMA-AHB (`ahb_dma`)
// register layout. The PAC's `dma` module maps to a different block (see
// SOC_DW_GDMA_SUPPORTED), so alias `ahb_dma` to keep ahb_v2.rs type paths working.
#[allow(unused)]
pub(crate) mod dma_compat {
    pub use super::pac::ahb_dma::*;
}

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x500D_6000;
    pub const INTERRUPT_MAP_BASE_APP_CPU: u32 = 0x500D_6800;
}

pub(crate) mod constants {
    #[allow(dead_code)] // used by other chips; reserved for future P4 DRAM boundary checks
    pub const SOC_DRAM_LOW: u32 = 0x4FF0_0000;
    #[allow(dead_code)]
    pub const SOC_DRAM_HIGH: u32 = 0x4FFC_0000;
}

pub(crate) fn pre_init() {
    // TODO: Check if anything needs to be done here
}

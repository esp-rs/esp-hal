use super::PeripheralInstance;
use crate::pac::hinf;

crate::any_peripheral! {
    /// Any SDIO HINF peripheral.
    pub peripheral AnyHinf<'d> {
        Hinf(crate::peripherals::HINF<'d>)
    }
}

/// Represents the HINF registers for SDIO peripherals.
pub struct HinfInfo {
    /// Represents the HINF register block.
    pub register_block: *const hinf::RegisterBlock,
}

unsafe impl Sync for HinfInfo {}

impl PeripheralInstance for AnyHinf<'_> {
    type Info = HinfInfo;

    fn info(&self) -> &'static Self::Info {
        static INFO: HinfInfo = HinfInfo {
            register_block: crate::peripherals::HINF::ptr(),
        };

        &INFO
    }
}

/// A peripheral singleton compatible with the SDIO HINF driver.
pub trait HinfInstance: PeripheralInstance + IntoAnyHinf {}

impl HinfInstance for AnyHinf<'_> {}

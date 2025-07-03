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

/// A peripheral singleton compatible with the SDIO HINF driver.
pub trait HinfInstance: any::Degrade {
    /// Gets a static reference the the [HinfInfo].
    fn info(&self) -> &'static HinfInfo {
        static INFO: HinfInfo = HinfInfo {
            register_block: crate::peripherals::HINF::ptr(),
        };

        &INFO
    }
}

impl HinfInstance for AnyHinf<'_> {}
impl HinfInstance for crate::peripherals::HINF<'_> {}

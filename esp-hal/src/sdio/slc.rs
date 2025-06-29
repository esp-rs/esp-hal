use crate::pac::slc;

crate::any_peripheral! {
    /// Any SDIO SLC peripheral.
    pub peripheral AnySlc<'d> {
        Slc(crate::peripherals::SLC<'d>)
    }
}

/// Represents the SLC registers for SDIO peripherals.
pub struct SlcInfo {
    /// Represents the SLC register block.
    pub register_block: *const slc::RegisterBlock,
}

unsafe impl Sync for SlcInfo {}

/// A peripheral singleton compatible with the SDIO SLC driver.
pub trait SlcInstance: any::Degrade {
    /// Gets a static reference the the [SlcInfo].
    fn info(&self) -> &'static SlcInfo {
        static INFO: SlcInfo = SlcInfo {
            register_block: crate::peripherals::SLC::ptr(),
        };

        &INFO
    }
}

impl SlcInstance for AnySlc<'_> {}
impl SlcInstance for crate::peripherals::SLC<'_> {}

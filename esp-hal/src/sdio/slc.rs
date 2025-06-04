use super::PeripheralInstance;
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

impl PeripheralInstance for AnySlc<'_> {
    type Info = SlcInfo;

    fn info(&self) -> &'static Self::Info {
        static INFO: SlcInfo = SlcInfo {
            register_block: crate::peripherals::SLC::ptr(),
        };

        &INFO
    }
}

/// A peripheral singleton compatible with the SDIO SLC driver.
pub trait SlcInstance: PeripheralInstance + IntoAnySlc {}

impl SlcInstance for AnySlc<'_> {}

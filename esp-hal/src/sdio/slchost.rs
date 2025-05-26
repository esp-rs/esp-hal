use super::PeripheralInstance;
use crate::pac::slchost;

crate::any_peripheral! {
    /// Any SDIO peripheral.
    pub peripheral AnySlchost<'d> {
        Slchost(crate::peripherals::SLCHOST<'d>)
    }
}

/// Represents the SLCHOST registers for SDIO peripherals.
pub struct SlchostInfo {
    /// Represents the SLCHOST register block.
    pub register_block: *const slchost::RegisterBlock,
}

unsafe impl Sync for SlchostInfo {}

impl PeripheralInstance for AnySlchost<'_> {
    type Info = SlchostInfo;

    fn info(&self) -> &'static Self::Info {
        static INFO: SlchostInfo = SlchostInfo {
            register_block: crate::peripherals::SLCHOST::ptr(),
        };

        &INFO
    }
}

/// A peripheral singleton compatible with the SDIO SLCHOST driver.
pub trait SlchostInstance: PeripheralInstance + IntoAnySlchost {}

impl SlchostInstance for AnySlchost<'_> {}

use crate::pac::slchost;

crate::any_peripheral! {
    /// Any SDIO SLCHOST peripheral.
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

/// A peripheral singleton compatible with the SDIO SLCHOST driver.
pub trait SlchostInstance: any::Degrade {
    /// Gets a static reference the the [SlchostInfo].
    fn info(&self) -> &'static SlchostInfo {
        static INFO: SlchostInfo = SlchostInfo {
            register_block: crate::peripherals::SLCHOST::ptr(),
        };

        &INFO
    }
}

impl SlchostInstance for AnySlchost<'_> {}
impl SlchostInstance for crate::peripherals::SLCHOST<'_> {}

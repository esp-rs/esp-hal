use crate::clock::ll::{ClockTree, SpiInstance};

impl SpiInstance {
    // SPI_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }
}

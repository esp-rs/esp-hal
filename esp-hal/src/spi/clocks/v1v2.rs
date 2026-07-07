use crate::clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance};

impl SpiInstance {
    // SPI_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<SpiFunctionClockConfig>,
        _new_config: SpiFunctionClockConfig,
    ) {
        // SPI is hardwired to APB; no clock source selection register.
    }
}

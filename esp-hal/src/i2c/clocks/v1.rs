use crate::clock::ll::{ClockTree, I2cFunctionClockConfig, I2cInstance};

impl I2cInstance {
    // I2C_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2cFunctionClockConfig>,
        _new_config: I2cFunctionClockConfig,
    ) {
        // I2C is hardwired to APB; no clock source selection register.
    }
}

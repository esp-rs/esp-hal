use crate::clock::ll::{ClockTree, I2cFunctionClockConfig, I2cFunctionClockSclk, I2cInstance};

impl I2cInstance {
    // I2C_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // No dedicated enable bit on ESP32-S2; clock selection via ref_always_on.
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2cFunctionClockConfig>,
        new_config: I2cFunctionClockConfig,
    ) {
        let regs = match self {
            I2cInstance::I2c0 => crate::peripherals::I2C0::regs(),
            I2cInstance::I2c1 => crate::peripherals::I2C1::regs(),
        };
        regs.ctr().modify(|_, w| {
            w.ref_always_on()
                .bit(!matches!(new_config.sclk(), I2cFunctionClockSclk::RefTick))
        });
    }
}

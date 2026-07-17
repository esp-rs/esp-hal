use crate::clock::ll::{ClockTree, I2cFunctionClockConfig, I2cFunctionClockSclk, I2cInstance};

impl I2cInstance {
    // I2C_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let regs = match self {
            #[cfg(soc_has_i2c0)]
            I2cInstance::I2c0 => crate::peripherals::I2C0::regs(),
            #[cfg(soc_has_i2c1)]
            I2cInstance::I2c1 => crate::peripherals::I2C1::regs(),
        };
        regs.clk_conf().modify(|_, w| w.sclk_active().bit(en));
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2cFunctionClockConfig>,
        new_config: I2cFunctionClockConfig,
    ) {
        let regs = match self {
            #[cfg(soc_has_i2c0)]
            I2cInstance::I2c0 => crate::peripherals::I2C0::regs(),
            #[cfg(soc_has_i2c1)]
            I2cInstance::I2c1 => crate::peripherals::I2C1::regs(),
        };
        // sclk_sel: 0 = XTAL, 1 = RC_FAST
        regs.clk_conf().modify(|_, w| unsafe {
            w.sclk_sel()
                .bit(matches!(new_config.sclk(), I2cFunctionClockSclk::RcFast));
            w.sclk_div_num().bits(new_config.div_num() as _)
        });
    }
}

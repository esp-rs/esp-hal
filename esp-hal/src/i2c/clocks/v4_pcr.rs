use crate::{
    clock::ll::{ClockTree, I2cFunctionClockConfig, I2cFunctionClockSclk, I2cInstance},
    peripherals::PCR,
};

impl I2cInstance {
    // I2C_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let reg = match self {
            #[cfg(soc_has_i2c0)]
            I2cInstance::I2c0 => PCR::regs().i2c_sclk_conf(0),
            #[cfg(soc_has_i2c1)]
            I2cInstance::I2c1 => PCR::regs().i2c_sclk_conf(1),
        };
        reg.modify(|_, w| w.i2c_sclk_en().bit(en));
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2cFunctionClockConfig>,
        new_config: I2cFunctionClockConfig,
    ) {
        let reg = match self {
            #[cfg(soc_has_i2c0)]
            I2cInstance::I2c0 => PCR::regs().i2c_sclk_conf(0),
            #[cfg(soc_has_i2c1)]
            I2cInstance::I2c1 => PCR::regs().i2c_sclk_conf(1),
        };
        reg.modify(|_, w| unsafe {
            w.i2c_sclk_sel()
                .bit(matches!(new_config.sclk(), I2cFunctionClockSclk::RcFast));
            w.i2c_sclk_div_num().bits(new_config.div_num() as _)
        });
    }
}

use crate::{
    clock::ll::{ClockTree, I2cFunctionClockConfig, I2cFunctionClockSclk, I2cInstance},
    peripherals::HP_SYS_CLKRST,
};

impl I2cInstance {
    // I2C_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // The function clock is gated by the peripheral clock control, which is
        // handled by the generated peripheral clock code.
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2cFunctionClockConfig>,
        new_config: I2cFunctionClockConfig,
    ) {
        let rc_fast = matches!(new_config.sclk(), I2cFunctionClockSclk::RcFast);
        let div_num = new_config.div_num() as u8;
        match self {
            #[cfg(soc_has_i2c0)]
            I2cInstance::I2c0 => {
                HP_SYS_CLKRST::regs().i2c0_ctrl0().modify(|_, w| unsafe {
                    w.i2c0_clk_src_sel().bit(rc_fast);
                    w.i2c0_clk_div_num().bits(div_num);
                    w.i2c0_clk_div_numerator().bits(0);
                    w.i2c0_clk_div_denominator().bits(0)
                });
            }
            #[cfg(soc_has_i2c1)]
            I2cInstance::I2c1 => {
                HP_SYS_CLKRST::regs().i2c1_ctrl0().modify(|_, w| unsafe {
                    w.i2c1_clk_src_sel().bit(rc_fast);
                    w.i2c1_clk_div_num().bits(div_num);
                    w.i2c1_clk_div_numerator().bits(0);
                    w.i2c1_clk_div_denominator().bits(0)
                });
            }
        }
    }
}

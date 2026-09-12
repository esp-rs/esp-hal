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
        HP_SYS_CLKRST::regs()
            .i2c_ctrl0(self as usize)
            .modify(|_, w| unsafe {
                w.clk_src_sel().bit(rc_fast);
                w.clk_div_num().bits(div_num);
                w.clk_div_numerator().bits(0);
                w.clk_div_denominator().bits(0)
            });
    }
}

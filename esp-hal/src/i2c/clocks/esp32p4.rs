use crate::{
    clock::ll::{ClockTree, I2cFunctionClockConfig, I2cFunctionClockSclk, I2cInstance},
    peripherals::HP_SYS_CLKRST,
};

impl I2cInstance {
    // I2C_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl10()
            .modify(|_, w| match self {
                I2cInstance::I2c0 => w.i2c0_clk_en().bit(en),
                I2cInstance::I2c1 => w.i2c1_clk_en().bit(en),
            });
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<I2cFunctionClockConfig>,
        new_config: I2cFunctionClockConfig,
    ) {
        let rc_fast = matches!(new_config.sclk(), I2cFunctionClockSclk::RcFast);
        match self {
            I2cInstance::I2c0 => {
                HP_SYS_CLKRST::regs()
                    .peri_clk_ctrl10()
                    .modify(|_, w| unsafe {
                        w.i2c0_clk_src_sel().bit(rc_fast);
                        w.i2c0_clk_div_num().bits(new_config.div_num() as _)
                    });
            }
            I2cInstance::I2c1 => {
                HP_SYS_CLKRST::regs()
                    .peri_clk_ctrl10()
                    .modify(|_, w| w.i2c1_clk_src_sel().bit(rc_fast));
                HP_SYS_CLKRST::regs()
                    .peri_clk_ctrl11()
                    .modify(|_, w| unsafe { w.i2c1_clk_div_num().bits(new_config.div_num() as _) });
            }
        }
    }
}

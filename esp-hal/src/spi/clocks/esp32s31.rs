use crate::{
    clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance},
    peripherals::HP_SYS_CLKRST,
};

impl SpiInstance {
    // SPI_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _enable: bool) {
        // SPI clock gates are managed by the peripheral clock infrastructure in system.rs
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<SpiFunctionClockConfig>,
        new_config: SpiFunctionClockConfig,
    ) {
        let source = match new_config {
            SpiFunctionClockConfig::Xtal => 0,
            SpiFunctionClockConfig::RcFast => 1,
            SpiFunctionClockConfig::Bbpll => 2,
        };

        // Programs `hs_clk_div_num`. The register value is divisor - 1, so 2 means /3
        // (480 MHz BBPLL -> 160 MHz). Reset the divider when the node is released so XTAL/RC_FAST
        // are not divided.
        let div_num = match new_config {
            SpiFunctionClockConfig::Xtal => 0,
            SpiFunctionClockConfig::RcFast => 0,
            SpiFunctionClockConfig::Bbpll => 2,
        };
        match self {
            SpiInstance::Spi2 => {
                HP_SYS_CLKRST::regs().gpspi2_ctrl0().modify(|_, w| unsafe {
                    w.clk_src_sel().bits(source);
                    w.hs_clk_div_num().bits(div_num)
                });
            }
            SpiInstance::Spi3 => {
                HP_SYS_CLKRST::regs().gpspi3_ctrl0().modify(|_, w| unsafe {
                    w.clk_src_sel().bits(source);
                    w.hs_clk_div_num().bits(div_num)
                });
            }
        }
    }
}

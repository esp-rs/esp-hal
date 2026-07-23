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
        match self {
            SpiInstance::Spi2 => {
                HP_SYS_CLKRST::regs()
                    .gpspi2_ctrl0()
                    .modify(|_, w| unsafe { w.gpspi2_clk_src_sel().bits(source) });
            }
            SpiInstance::Spi3 => {
                HP_SYS_CLKRST::regs()
                    .gpspi3_ctrl0()
                    .modify(|_, w| unsafe { w.gpspi3_clk_src_sel().bits(source) });
            }
        }
    }
}

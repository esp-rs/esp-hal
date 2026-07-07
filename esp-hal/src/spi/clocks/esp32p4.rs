use crate::{
    clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance},
    peripherals::HP_SYS_CLKRST,
};

impl SpiInstance {
    // SPI_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // SPI clock gates are managed by the peripheral clock infrastructure in system.rs.
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
            // SDIO_PLL0
            // APLL
            SpiFunctionClockConfig::Spll => 4,
        };
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl116()
            .modify(|_, w| unsafe {
                match self {
                    Self::Spi2 => w.gpspi2_clk_src_sel().bits(source),
                    Self::Spi3 => w.gpspi3_clk_src_sel().bits(source),
                }
            });
    }
}

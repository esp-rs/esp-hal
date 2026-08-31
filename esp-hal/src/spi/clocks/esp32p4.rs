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

        // Programs `gpspi*_hs_clk_div_num`. The register value is divisor - 1, so 2 means /3
        // (480 MHz SPLL -> 160 MHz). Reset the divider when the node is released so XTAL/RC_FAST
        // are not divided.
        let div_num = match new_config {
            SpiFunctionClockConfig::Xtal => 0,
            SpiFunctionClockConfig::RcFast => 0,
            // SDIO_PLL0
            // APLL
            SpiFunctionClockConfig::Spll => 2,
        };
        match self {
            Self::Spi2 => {
                HP_SYS_CLKRST::regs()
                    .peri_clk_ctrl116()
                    .modify(|_, w| unsafe {
                        w.gpspi2_clk_src_sel().bits(source);
                        w.gpspi2_hs_clk_div_num().bits(div_num)
                    });
            }
            Self::Spi3 => {
                HP_SYS_CLKRST::regs()
                    .peri_clk_ctrl116()
                    .modify(|_, w| unsafe { w.gpspi3_clk_src_sel().bits(source) });
                HP_SYS_CLKRST::regs()
                    .peri_clk_ctrl117()
                    .modify(|_, w| unsafe { w.gpspi3_hs_clk_div_num().bits(div_num) });
            }
        }
    }
}

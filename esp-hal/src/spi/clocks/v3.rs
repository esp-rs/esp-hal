use crate::clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance};

impl SpiInstance {
    // SPI_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<SpiFunctionClockConfig>,
        new_config: SpiFunctionClockConfig,
    ) {
        let regs = match self {
            #[cfg(soc_has_spi2)]
            SpiInstance::Spi2 => crate::peripherals::SPI2::regs(),
            #[cfg(soc_has_spi3)]
            SpiInstance::Spi3 => crate::peripherals::SPI3::regs(),
        };
        regs.clk_gate().modify(|_, w| {
            let bit = cfg_select! {
                esp32c2 => matches!(new_config, SpiFunctionClockConfig::Pll40m),
                esp32c3 => matches!(new_config, SpiFunctionClockConfig::Pll80m),
                esp32s3 => matches!(new_config, SpiFunctionClockConfig::Apb),
            };
            w.mst_clk_sel().bit(bit)
        });
    }
}

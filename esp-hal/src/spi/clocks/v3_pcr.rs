use crate::{
    clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance},
    peripherals::PCR,
};

impl SpiInstance {
    // SPI_FUNCTION_CLOCK

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        match self {
            SpiInstance::Spi2 => PCR::regs()
                .spi2_clkm_conf()
                .modify(|_, w| w.spi2_clkm_en().bit(en)),
            #[cfg(esp32h4)]
            SpiInstance::Spi3 => PCR::regs()
                .spi3_clkm_conf()
                .modify(|_, w| w.spi3_clkm_en().bit(en)),
        };
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<SpiFunctionClockConfig>,
        new_config: SpiFunctionClockConfig,
    ) {
        let sel = match new_config {
            SpiFunctionClockConfig::Xtal => 0,
            #[cfg(esp32c6)]
            SpiFunctionClockConfig::PllF80m => 1,
            #[cfg(any(esp32h2, esp32h4))]
            SpiFunctionClockConfig::PllF48m => 1,
            #[cfg(any(esp32c5, esp32c61))]
            SpiFunctionClockConfig::PllF160m => 1,
            SpiFunctionClockConfig::RcFast => 2,
            #[cfg(esp32c5)]
            SpiFunctionClockConfig::PllF120m => 3,
        };

        match self {
            SpiInstance::Spi2 => PCR::regs()
                .spi2_clkm_conf()
                .modify(|_, w| unsafe { w.spi2_clkm_sel().bits(sel) }),
            #[cfg(esp32h4)]
            SpiInstance::Spi3 => PCR::regs()
                .spi3_clkm_conf()
                .modify(|_, w| unsafe { w.spi3_clkm_sel().bits(sel) }),
        };
    }
}

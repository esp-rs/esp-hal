use crate::{
    clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance},
    peripherals::PCR,
};

impl SpiInstance {
    #[cfg(soc_has_clock_node_spi_pll_f160m_div2)]
    pub(crate) fn enable_pll_f160m_div2_impl(self, _clocks: &mut ClockTree, _en: bool) {}

    #[cfg(soc_has_clock_node_spi_pll_f120m_div2)]
    pub(crate) fn enable_pll_f120m_div2_impl(self, _clocks: &mut ClockTree, _en: bool) {}

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let regs = PCR::regs();
        match self {
            Self::Spi2 => regs
                .spi2_clkm_conf()
                .modify(|_, w| w.spi2_clkm_en().bit(en)),
            #[cfg(soc_has_spi3)]
            Self::Spi3 => regs
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
        // Dividing the 120 MHz and 160 MHz PLL sources by two keeps the master clock within 80 MHz.
        // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_hal_gpspi/esp32c5/include/hal/spi_ll.h#L178-L195>.
        let (source, _pre_div_num) = match new_config {
            SpiFunctionClockConfig::Xtal => (0, 0),
            #[cfg(esp32c6)]
            SpiFunctionClockConfig::PllF80m => (1, 0),
            #[cfg(any(esp32h2, esp32h4))]
            SpiFunctionClockConfig::PllF48m => (1, 0),
            #[cfg(soc_has_clock_node_spi_pll_f160m_div2)]
            SpiFunctionClockConfig::PllF160m => (1, 1),
            SpiFunctionClockConfig::RcFast => (2, 0),
            #[cfg(soc_has_clock_node_spi_pll_f120m_div2)]
            SpiFunctionClockConfig::PllF120m => (3, 1),
        };

        let regs = PCR::regs();
        match self {
            Self::Spi2 => regs.spi2_clkm_conf().modify(|_, w| unsafe {
                w.spi2_clkm_sel().bits(source);
                #[cfg(spi_master_has_clk_pre_div)]
                w.spi2_clkm_div_num().bits(_pre_div_num);
                w
            }),
            #[cfg(soc_has_spi3)]
            Self::Spi3 => regs.spi3_clkm_conf().modify(|_, w| unsafe {
                w.spi3_clkm_sel().bits(source);
                #[cfg(spi_master_has_clk_pre_div)]
                w.spi3_clkm_div_num().bits(_pre_div_num);
                w
            }),
        };
    }
}

use crate::{
    clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance},
    peripherals::HP_SYS_CLKRST,
};

impl SpiInstance {
    pub(crate) fn enable_bbpll_mst_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _enable: bool) {
        // The high-speed and master gates are enabled at reset.
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<SpiFunctionClockConfig>,
        new_config: SpiFunctionClockConfig,
    ) {
        // The high-speed and master clocks are limited to 160 MHz and 80 MHz, respectively.
        // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_hal_gpspi/esp32s31/include/hal/spi_ll.h#L228-L248>.
        let (source, hs_div_num, mst_div_num) = match new_config {
            SpiFunctionClockConfig::Xtal => (0, 0, 0),
            SpiFunctionClockConfig::RcFast => (1, 0, 0),
            SpiFunctionClockConfig::Bbpll => (2, 2, 1),
        };
        let regs = HP_SYS_CLKRST::regs();
        match self {
            Self::Spi2 => {
                regs.gpspi2_ctrl0().modify(|_, w| unsafe {
                    w.clk_src_sel().bits(source);
                    w.hs_clk_div_num().bits(hs_div_num);
                    w.mst_clk_div_num().bits(mst_div_num)
                });
            }
            Self::Spi3 => {
                regs.gpspi3_ctrl0().modify(|_, w| unsafe {
                    w.clk_src_sel().bits(source);
                    w.hs_clk_div_num().bits(hs_div_num);
                    w.mst_clk_div_num().bits(mst_div_num)
                });
            }
        }
    }
}

use crate::{
    clock::ll::{ClockTree, SpiFunctionClockConfig, SpiInstance},
    peripherals::HP_SYS_CLKRST,
};

impl SpiInstance {
    pub(crate) fn enable_spll_mst_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}

    pub(crate) fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let regs = HP_SYS_CLKRST::regs();
        match self {
            Self::Spi2 => {
                regs.peri_clk_ctrl116().modify(|_, w| {
                    w.gpspi2_hs_clk_en().bit(en);
                    w.gpspi2_mst_clk_en().bit(en)
                });
            }
            Self::Spi3 => {
                regs.peri_clk_ctrl116()
                    .modify(|_, w| w.gpspi3_hs_clk_en().bit(en));
                regs.peri_clk_ctrl117()
                    .modify(|_, w| w.gpspi3_mst_clk_en().bit(en));
            }
        }
    }

    pub(crate) fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<SpiFunctionClockConfig>,
        new_config: SpiFunctionClockConfig,
    ) {
        // The high-speed and master clocks are limited to 160 MHz and 80 MHz, respectively.
        // See <https://github.com/espressif/esp-idf/blob/0e6b480f858c69862ffb6001d96e9eef74599a6e/components/esp_hal_gpspi/esp32p4/include/hal/spi_ll.h#L228-L248>.
        let (source, hs_div_num, mst_div_num) = match new_config {
            SpiFunctionClockConfig::Xtal => (0, 0, 0),
            SpiFunctionClockConfig::RcFast => (1, 0, 0),
            SpiFunctionClockConfig::Spll => (4, 2, 1),
        };
        let regs = HP_SYS_CLKRST::regs();
        match self {
            Self::Spi2 => {
                regs.peri_clk_ctrl116().modify(|_, w| unsafe {
                    w.gpspi2_clk_src_sel().bits(source);
                    w.gpspi2_hs_clk_div_num().bits(hs_div_num);
                    w.gpspi2_mst_clk_div_num().bits(mst_div_num)
                });
            }
            Self::Spi3 => {
                let select_source = || {
                    regs.peri_clk_ctrl116()
                        .modify(|_, w| unsafe { w.gpspi3_clk_src_sel().bits(source) });
                };
                let set_dividers = || {
                    regs.peri_clk_ctrl117().modify(|_, w| unsafe {
                        w.gpspi3_hs_clk_div_num().bits(hs_div_num);
                        w.gpspi3_mst_clk_div_num().bits(mst_div_num)
                    });
                };

                // Keep SPI3 within its clock limits while switching sources.
                if new_config == SpiFunctionClockConfig::Spll {
                    set_dividers();
                    select_source();
                } else {
                    select_source();
                    set_dividers();
                }
            }
        }
    }
}

use crate::clock::ll::{ClockTree, TsensInstance, TsensSclkConfig};

impl TsensInstance {
    pub(crate) fn enable_sclk_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // The TSENS peripheral guard manages the clock gate.
    }

    pub(crate) fn configure_sclk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TsensSclkConfig>,
        new_config: TsensSclkConfig,
    ) {
        let xtal = matches!(new_config, TsensSclkConfig::Xtal);
        cfg_select! {
            esp32c3 => {
                crate::peripherals::APB_SARADC::regs()
                    .tsens_ctrl2()
                    .modify(|_, w| w.clk_sel().bit(xtal));
            }
            esp32c6 => {
                // https://github.com/espressif/esp-idf/blob/ab95e70748f2eddac6f87217b0cf62533684a6ba/components/esp_hal_ana_conv/esp32c6/include/hal/temperature_sensor_ll.h#L99-L114
                crate::peripherals::PCR::regs()
                    .tsens_clk_conf()
                    .modify(|_, w| w.tsens_clk_sel().bit(xtal));
            }
        }
    }
}

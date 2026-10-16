use crate::{
    clock::ll::{ClockTree, TsensInstance, TsensSclkConfig},
    peripherals::APB_SARADC,
};

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
        APB_SARADC::regs()
            .tsens_ctrl2()
            .write(|w| w.clk_sel().bit(matches!(new_config, TsensSclkConfig::Xtal)));
    }
}

use crate::soc::regi2c;

const SYSTEM_WIFI_CLK_RNG_EN: u32 = 1 << 15;

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    unsafe {
        let rtc_cntl = crate::peripherals::LPWR::regs();
        let system = crate::peripherals::SYSTEM::regs();
        let apb_saradc = crate::peripherals::APB_SARADC::regs();
        let sens = crate::peripherals::SENS::regs();
        let syscon = crate::peripherals::APB_CTRL::regs();

        // `wifi_clk_en` register is defined in a really weird way, for now just simple
        // bit edit
        syscon.wifi_clk_en().modify(|r, w| {
            let current_bits = r.bits();
            let new_bits = current_bits | SYSTEM_WIFI_CLK_RNG_EN;
            w.bits(new_bits)
        });

        // Enable 8M clock source for RNG (this is actually enough to produce strong
        // random results, but enabling the SAR ADC as well adds some insurance.)
        rtc_cntl
            .clk_conf()
            .modify(|_, w| w.dig_clk8m_en().set_bit());

        // Enable SAR ADC to read a disconnected input for additional entropy
        // Reset ADC clock
        system
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().set_bit());

        system
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().clear_bit());

        apb_saradc.clkm_conf().modify(|_, w| w.clk_sel().bits(2));
        apb_saradc.ctrl().modify(|_, w| w.sar_clk_gated().set_bit());
        apb_saradc.clkm_conf().modify(|_, w| w.clk_en().set_bit());

        apb_saradc
            .clkm_conf()
            .modify(|_, w| w.clkm_div_num().bits(3));

        apb_saradc.ctrl().modify(|_, w| w.sar_clk_div().bits(3));
        apb_saradc.ctrl2().modify(|_, w| w.timer_target().bits(3));
        apb_saradc.ctrl().modify(|_, w| w.sar_clk_div().bits(3));

        sens.sar_power_xpd_sar()
            .modify(|_, w| w.force_xpd_sar().bits(3));

        apb_saradc
            .ctrl2()
            .modify(|_, w| w.meas_num_limit().clear_bit());

        apb_saradc.ctrl().modify(|_, w| w.work_mode().bits(1));
        apb_saradc.ctrl().modify(|_, w| w.sar2_patt_len().bits(0));
        apb_saradc.sar2_patt_tab1().modify(|_, w| w.bits(0xafffff));
        apb_saradc.ctrl().modify(|_, w| w.sar1_patt_len().bits(0));
        apb_saradc.sar1_patt_tab1().modify(|_, w| w.bits(0xafffff));

        sens.sar_meas1_mux()
            .modify(|_, w| w.sar1_dig_force().set_bit());

        sens.sar_meas2_mux()
            .modify(|_, w| w.sar2_rtc_force().clear_bit());

        apb_saradc
            .arb_ctrl()
            .modify(|_, w| w.grant_force().clear_bit());

        apb_saradc
            .arb_ctrl()
            .modify(|_, w| w.fix_priority().clear_bit());

        apb_saradc
            .filter_ctrl0()
            .modify(|_, w| w.filter_channel0().bits(0xD));

        apb_saradc
            .filter_ctrl0()
            .modify(|_, w| w.filter_channel1().bits(0xD));

        apb_saradc.ctrl2().modify(|_, w| w.timer_sel().set_bit());
        apb_saradc.ctrl2().modify(|_, w| w.timer_en().set_bit());

        regi2c::ADC_SAR1_ENCAL_REF.write_field(1);
        regi2c::ADC_SAR_ENT_TSENS.write_field(1);
        regi2c::ADC_SAR_ENT_RTC.write_field(1);
        regi2c::ADC_SAR_DTEST_RTC.write_field(1);
    }
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    let system = crate::peripherals::SYSTEM::regs();
    let apb_saradc = crate::peripherals::APB_SARADC::regs();
    let sens = crate::peripherals::SENS::regs();

    unsafe {
        regi2c::ADC_SAR1_ENCAL_REF.write_field(0);
        regi2c::ADC_SAR_ENT_TSENS.write_field(0);
        regi2c::ADC_SAR_ENT_RTC.write_field(0);
        regi2c::ADC_SAR_DTEST_RTC.write_field(0);

        sens.sar_power_xpd_sar()
            .modify(|_, w| w.force_xpd_sar().bits(0));

        apb_saradc.ctrl2().modify(|_, w| w.timer_en().clear_bit());

        system
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().clear_bit());

        system
            .perip_rst_en0()
            .modify(|_, w| w.apb_saradc_rst().set_bit());
    }
}

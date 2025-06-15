use crate::{
    peripherals::{APB_SARADC, LPWR, SYSTEM},
    soc::regi2c,
};

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    let rtc_cntl = LPWR::regs();
    let system = SYSTEM::regs();
    let apb_saradc = APB_SARADC::regs();

    unsafe {
        // RNG module is always clock enabled
        rtc_cntl
            .cntl_sensor_ctrl()
            .modify(|_, w| w.force_xpd_sar().bits(3));

        rtc_cntl.ana_conf().modify(|_, w| w.sar_i2c_pu().set_bit());

        // Bridging sar2 internal reference voltage
        // Cannot replace with PAC-based functions
        regi2c::ADC_SAR2_ENCAL_REF.write_field(1);
        regi2c::ADC_SAR_DTEST_RTC.write_field(0);
        regi2c::ADC_SAR_ENT_RTC.write_field(0);
        regi2c::ADC_SAR_ENT_TSENS.write_field(0);

        // Enable SAR ADC2 internal channel to read adc2 ref voltage for additional
        // entropy
        system
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().set_bit());

        system
            .perip_rst_en0()
            .modify(|_, w| w.apb_saradc_rst().clear_bit());

        apb_saradc.clkm_conf().modify(|_, w| w.clk_sel().bits(2));
        apb_saradc.clkm_conf().modify(|_, w| w.clk_en().set_bit());
        apb_saradc.ctrl().modify(|_, w| w.sar_clk_gated().set_bit());
        apb_saradc.ctrl().modify(|_, w| w.xpd_sar_force().bits(3));
        apb_saradc.ctrl().modify(|_, w| w.sar_clk_div().bits(1));
        apb_saradc.fsm_wait().modify(|_, w| w.rstb_wait().bits(8));
        apb_saradc.fsm_wait().modify(|_, w| w.xpd_wait().bits(5));

        apb_saradc
            .fsm_wait()
            .modify(|_, w| w.standby_wait().bits(100));

        apb_saradc
            .ctrl()
            .modify(|_, w| w.sar_patt_p_clear().set_bit());

        apb_saradc
            .ctrl()
            .modify(|_, w| w.sar_patt_p_clear().clear_bit());

        apb_saradc.ctrl().modify(|_, w| w.sar_patt_len().bits(0));

        apb_saradc
            .sar_patt_tab1()
            .modify(|_, w| w.sar_patt_tab1().bits(0x9cffff));

        apb_saradc
            .sar_patt_tab2()
            .modify(|_, w| w.sar_patt_tab2().bits(0x9cffff));

        apb_saradc.ctrl2().modify(|_, w| w.timer_target().bits(100));

        apb_saradc
            .clkm_conf()
            .modify(|_, w| w.clkm_div_num().bits(15));

        apb_saradc
            .ctrl2()
            .modify(|_, w| w.meas_num_limit().clear_bit());

        apb_saradc.dma_conf().modify(|_, w| w.adc_trans().set_bit());

        apb_saradc.ctrl2().modify(|_, w| w.timer_en().set_bit());
    }
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    let apb_saradc = APB_SARADC::regs();
    let rtc_cntl = LPWR::regs();

    unsafe {
        regi2c::ADC_SAR2_ENCAL_REF.write_field(0);

        apb_saradc.ctrl2().modify(|_, w| w.timer_en().clear_bit());

        apb_saradc
            .dma_conf()
            .modify(|_, w| w.adc_trans().clear_bit());

        apb_saradc
            .sar_patt_tab1()
            .modify(|_, w| w.sar_patt_tab1().bits(0xffffff));

        apb_saradc
            .sar_patt_tab2()
            .modify(|_, w| w.sar_patt_tab2().bits(0xffffff));

        apb_saradc.clkm_conf().modify(|_, w| w.clk_en().clear_bit());

        apb_saradc.ctrl().modify(|_, w| w.xpd_sar_force().bits(0));

        rtc_cntl
            .cntl_sensor_ctrl()
            .modify(|_, w| w.force_xpd_sar().bits(0));
    }
}

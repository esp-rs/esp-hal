use crate::{
    peripherals::{APB_SARADC, I2C_ANA_MST, LPWR, SENS, SYSTEM},
    soc::regi2c,
};

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    // Enable 8M clock source for RNG (this is actually enough to produce strong
    // random results, but enabling the SAR ADC as well adds some insurance.)
    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.dig_clk8m_en().set_bit());

    // Enable SAR ADC to read a disconnected input for additional entropy
    SYSTEM::regs()
        .perip_clk_en0()
        .modify(|_, w| w.apb_saradc_clk_en().set_bit());

    APB_SARADC::regs()
        .clkm_conf()
        .modify(|_, w| unsafe { w.clk_sel().bits(2) });

    LPWR::regs().ana_conf().modify(|_, w| {
        w.sar_i2c_force_pd().clear_bit();
        w.sar_i2c_force_pu().set_bit()
    });

    I2C_ANA_MST::regs()
        .config1()
        .modify(|_, w| w.sar().clear_bit());
    I2C_ANA_MST::regs()
        .config0() // ANA_CONFIG2_REG in esp-idf
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 16)) });

    regi2c::ADC_SAR_DTEST_RTC.write_field(0);
    regi2c::ADC_SAR_ENT_TSENS.write_field(1);
    regi2c::ADC_SAR1_ENCAL_REF.write_field(1);
    regi2c::ADC_SAR1_DREF.write_field(4);
    regi2c::ADC_SAR2_DREF.write_field(4);
    regi2c::ADC_SAR_ENT_RTC.write_field(0);

    APB_SARADC::regs()
        .ctrl()
        .modify(|_, w| unsafe { w.sar1_patt_len().bits(0) });

    APB_SARADC::regs()
        .sar1_patt_tab1()
        .modify(|_, w| unsafe { w.bits(0xafffffff) });

    APB_SARADC::regs()
        .ctrl()
        .modify(|_, w| unsafe { w.sar2_patt_len().bits(0) });

    APB_SARADC::regs()
        .sar2_patt_tab1()
        .modify(|_, w| unsafe { w.bits(0xafffffff) });

    SENS::regs()
        .sar_meas1_mux()
        .modify(|_, w| w.sar1_dig_force().set_bit());

    APB_SARADC::regs()
        .ctrl()
        .modify(|_, w| unsafe { w.work_mode().bits(1) });

    APB_SARADC::regs()
        .ctrl2()
        .modify(|_, w| w.meas_num_limit().clear_bit());

    SENS::regs()
        .sar_power_xpd_sar()
        .modify(|_, w| unsafe { w.force_xpd_sar().bits(3) });

    APB_SARADC::regs().ctrl2().modify(|_, w| unsafe {
        w.timer_sel().set_bit();
        w.timer_target().bits(100)
    });
    APB_SARADC::regs()
        .ctrl()
        .modify(|_, w| w.start_force().clear_bit());
    APB_SARADC::regs()
        .ctrl2()
        .modify(|_, w| w.timer_en().set_bit());
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    unsafe {
        // Restore internal I2C bus state
        regi2c::ADC_SAR1_DREF.write_field(1);
        regi2c::ADC_SAR2_DREF.write_field(1);
        regi2c::ADC_SAR1_ENCAL_REF.write_field(0);
        regi2c::ADC_SAR_ENT_TSENS.write_field(0);
        regi2c::ADC_SAR_ENT_RTC.write_field(0);

        // Restore SARADC to default mode
        SENS::regs()
            .sar_meas1_mux()
            .modify(|_, w| w.sar1_dig_force().clear_bit());

        SYSTEM::regs()
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().set_bit());

        SENS::regs()
            .sar_power_xpd_sar()
            .modify(|_, w| w.force_xpd_sar().bits(0));

        APB_SARADC::regs()
            .ctrl2()
            .modify(|_, w| w.timer_en().clear_bit());
    }
}

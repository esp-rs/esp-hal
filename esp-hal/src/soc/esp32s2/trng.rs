use crate::{
    peripherals::{APB_SARADC, LPWR, SENS, SYSTEM},
    soc::regi2c,
};

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    let rtc_cntl = LPWR::regs();
    let dport = SYSTEM::regs();
    let apb_saradc = APB_SARADC::regs();
    let sens = SENS::regs();

    unsafe {
        // Enable 8M clock source for RNG (this is actually enough to produce strong
        // random results, but enabling the SAR ADC as well adds some insurance.)
        rtc_cntl
            .clk_conf()
            .modify(|_, w| w.dig_clk8m_en().set_bit());

        // Enable SAR ADC to read a disconnected input for additional entropy
        dport
            .perip_clk_en0()
            .modify(|_, w| w.apb_saradc_clk_en().set_bit());

        apb_saradc.clkm_conf().modify(|_, w| w.clk_sel().bits(2));

        rtc_cntl.ana_conf().modify(|_, w| {
            w.sar_i2c_force_pd().clear_bit();
            w.sar_i2c_force_pu().set_bit()
        });

        // Temporarily not in PACs
        // esp-idf/components/soc/esp32s2/include/soc/regi2c_defs.h

        const ANA_CONFIG_REG: u32 = 0x6000E044;
        const ANA_CONFIG2_REG: u32 = 0x6000E048;

        clear_peri_reg_mask(ANA_CONFIG_REG, 1 << 18);
        set_peri_reg_mask(ANA_CONFIG2_REG, 1 << 16);

        regi2c::ADC_SAR1_DREF.write_field(4);
        regi2c::ADC_SAR2_DREF.write_field(4);
        regi2c::ADC_SAR1_ENCAL_REF.write_field(1);
        regi2c::ADC_SAR_ENT_TSENS.write_field(1);
        regi2c::ADC_SAR_ENT_RTC.write_field(0);

        apb_saradc.ctrl().modify(|_, w| w.sar1_patt_len().bits(0));

        apb_saradc
            .sar1_patt_tab1()
            .modify(|_, w| w.bits(0xafffffff));

        apb_saradc.ctrl().modify(|_, w| w.sar2_patt_len().bits(0));

        apb_saradc
            .sar2_patt_tab1()
            .modify(|_, w| w.bits(0xafffffff));

        sens.sar_meas1_mux()
            .modify(|_, w| w.sar1_dig_force().set_bit());

        apb_saradc.ctrl().modify(|_, w| w.work_mode().bits(1));

        apb_saradc
            .ctrl2()
            .modify(|_, w| w.meas_num_limit().clear_bit());

        sens.sar_power_xpd_sar()
            .modify(|_, w| w.force_xpd_sar().bits(3));

        apb_saradc.ctrl2().modify(|_, w| {
            w.timer_sel().set_bit();
            w.timer_target().bits(100)
        });
        apb_saradc.ctrl().modify(|_, w| w.start_force().clear_bit());
        apb_saradc.ctrl2().modify(|_, w| w.timer_en().set_bit());
    }
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

fn set_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
    }
}

fn clear_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
    }
}

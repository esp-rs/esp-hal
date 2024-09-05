const SYSTEM_WIFI_CLK_RNG_EN: u32 = 1 << 15;

const I2C_SAR_ADC: u8 = 0x69;
const I2C_SAR_ADC_HOSTID: u8 = 1;
const ADC_SARADC_ENCAL_REF_ADDR: u8 = 0x7;
const ADC_SARADC_ENCAL_REF_ADDR_MSB: u32 = 4;
const ADC_SARADC_ENCAL_REF_ADDR_LSB: u32 = 4;
const ADC_SARADC_ENT_TSENS_ADDR: u32 = 0x7;
const ADC_SARADC_ENT_TSENS_ADDR_MSB: u32 = 2;
const ADC_SARADC_ENT_TSENS_ADDR_LSB: u32 = 2;
const ADC_SARADC_ENT_RTC_ADDR: u32 = 0x7;
const ADC_SARADC_ENT_RTC_ADDR_MSB: u32 = 3;
const ADC_SARADC_ENT_RTC_ADDR_LSB: u32 = 3;
const ADC_SARADC_DTEST_RTC_ADDR: u32 = 0x7;
const ADC_SARADC_DTEST_RTC_ADDR_MSB: u32 = 1;
const ADC_SARADC_DTEST_RTC_ADDR_LSB: u32 = 0;

use crate::regi2c_write_mask;

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    unsafe {
        let rtc_cntl = &*crate::peripherals::RTC_CNTL::ptr();
        let system = &*crate::peripherals::SYSTEM::ptr();
        let apb_saradc = &*crate::peripherals::APB_SARADC::ptr();
        let sens = &*crate::peripherals::SENS::ptr();
        let syscon = &*crate::peripherals::APB_CTRL::ptr();

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

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENCAL_REF_ADDR, 1);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_TSENS_ADDR, 1);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_RTC_ADDR, 1);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_DTEST_RTC_ADDR, 1);
    }
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    let system = unsafe { &*crate::peripherals::SYSTEM::ptr() };
    let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::ptr() };
    let sens = unsafe { &*crate::peripherals::SENS::ptr() };

    unsafe {
        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENCAL_REF_ADDR, 0);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_TSENS_ADDR, 0);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_RTC_ADDR, 0);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_DTEST_RTC_ADDR, 0);

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

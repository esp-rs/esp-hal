const I2C_SAR_ADC: u8 = 0x69;
const I2C_SAR_ADC_HOSTID: u8 = 0;
const ADC_SARADC2_ENCAL_REF_ADDR: u8 = 0x7;
const ADC_SARADC2_ENCAL_REF_ADDR_MSB: u8 = 6;
const ADC_SARADC2_ENCAL_REF_ADDR_LSB: u8 = 6;
const ADC_SARADC_DTEST_RTC_ADDR: u8 = 0x7;
const ADC_SARADC_DTEST_RTC_ADDR_MSB: u8 = 1;
const ADC_SARADC_DTEST_RTC_ADDR_LSB: u8 = 0;
const ADC_SARADC_ENT_RTC_ADDR: u8 = 0x7;
const ADC_SARADC_ENT_RTC_ADDR_MSB: u8 = 3;
const ADC_SARADC_ENT_RTC_ADDR_LSB: u8 = 3;
const ADC_SARADC_ENT_TSENS_ADDR: u8 = 0x07;
const ADC_SARADC_ENT_TSENS_ADDR_MSB: u8 = 2;
const ADC_SARADC_ENT_TSENS_ADDR_LSB: u8 = 2;

use crate::rom::regi2c_write_mask;

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };
    let system = unsafe { &*crate::peripherals::SYSTEM::ptr() };
    let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::ptr() };

    unsafe {
        // RNG module is always clock enabled
        rtc_cntl
            .cntl_sensor_ctrl()
            .modify(|_, w| w.force_xpd_sar().bits(3));

        rtc_cntl.ana_conf().modify(|_, w| w.sar_i2c_pu().set_bit());

        // Bridging sar2 internal reference voltage
        // Cannot replace with PAC-based functions
        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC2_ENCAL_REF_ADDR, 1);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_DTEST_RTC_ADDR, 0);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_RTC_ADDR, 0);

        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_TSENS_ADDR, 0);

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
    let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::ptr() };
    let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };

    unsafe {
        regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC2_ENCAL_REF_ADDR, 0);

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

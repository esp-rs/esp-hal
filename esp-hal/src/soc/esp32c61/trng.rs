use crate::{
    peripherals::{APB_SARADC, PCR, PMU, RNG},
    soc::regi2c,
};

// `I2C_SAR_ADC_INIT_CODE_VAL`
const SAR_ADC_INIT_CODE: u16 = 2150;

// `RNG_LL_CFG_PSCALE`
const RNG_TIMER_PSCALE: u8 = 255;

// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/hal/esp32c61/include/hal/rng_ll.h#L88-L95>
pub(crate) fn rng_ll_enable() {
    let rng = RNG::regs();
    rng.clk_en().modify(|_, w| w.rng_ck_en().set_bit());
    rng.rng_cfg().modify(|_, w| unsafe {
        w.rng_timer_pscale().bits(RNG_TIMER_PSCALE);
        w.rng_sample_enable().set_bit();
        w.rtc_timer_en().enable();
        w.rng_timer_en().set_bit()
    });
}

/// Enables true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32c61.c#L21-L69>
pub(crate) fn ensure_randomness() {
    let pcr = PCR::regs();
    let apb_saradc = APB_SARADC::regs();

    // `sar_periph_ctrl_adc_reset`: the ADC reset also clears the temperature sensor registers
    let tsens_ctrl = apb_saradc.tsens_ctrl().read().bits();
    let tsens_ctrl2 = apb_saradc.tsens_ctrl2().read().bits();
    let tsens_wake = apb_saradc.tsens_wake().read().bits();
    let tsens_sample = apb_saradc.tsens_sample().read().bits();

    pcr.saradc_conf().modify(|_, w| w.saradc_rst_en().set_bit());
    pcr.saradc_conf()
        .modify(|_, w| w.saradc_rst_en().clear_bit());
    pcr.saradc_conf()
        .modify(|_, w| w.saradc_reg_rst_en().set_bit());
    pcr.saradc_conf()
        .modify(|_, w| w.saradc_reg_rst_en().clear_bit());

    pcr.tsens_clk_conf()
        .modify(|_, w| w.tsens_rst_en().set_bit());
    pcr.tsens_clk_conf()
        .modify(|_, w| w.tsens_rst_en().clear_bit());
    apb_saradc
        .tsens_ctrl()
        .write(|w| unsafe { w.bits(tsens_ctrl) });
    apb_saradc
        .tsens_ctrl2()
        .write(|w| unsafe { w.bits(tsens_ctrl2) });
    apb_saradc
        .tsens_wake()
        .write(|w| unsafe { w.bits(tsens_wake) });
    apb_saradc
        .tsens_sample()
        .write(|w| unsafe { w.bits(tsens_sample) });

    pcr.saradc_conf()
        .modify(|_, w| w.saradc_reg_clk_en().set_bit());
    pcr.saradc_clkm_conf().modify(|_, w| unsafe {
        w.saradc_clkm_en().set_bit();
        w.saradc_clkm_sel().xtal();
        w.saradc_clkm_div_num().bits(0);
        w.saradc_clkm_div_b().bits(0);
        w.saradc_clkm_div_a().bits(0)
    });
    apb_saradc.ctrl().modify(|_, w| w.sar_clk_gated().set_bit());

    // `regi2c_ctrl_ll_i2c_sar_periph_enable`
    let pmu = PMU::regs();
    pmu.rf_pwc().modify(|_, w| w.perif_i2c_rstb().clear_bit());
    crate::rom::ets_delay_us(1);
    pmu.rf_pwc().modify(|_, w| w.xpd_perif_i2c().set_bit());
    pmu.rf_pwc().modify(|_, w| w.perif_i2c_rstb().set_bit());

    regi2c::ADC_SAR_DTEST_RTC.write_field(0);
    regi2c::ADC_SAR_ENT_RTC.write_field(1);
    regi2c::ADC_SAR1_ENCAL_REF.write_field(1);
    regi2c::ADC_SAR2_ENCAL_REF.write_field(1);

    let [init_code_high, init_code_low] = SAR_ADC_INIT_CODE.to_be_bytes();
    regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(init_code_high);
    regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(init_code_low);
    regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(init_code_high);
    regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(init_code_low);

    apb_saradc.sar_patt_tab1().modify(|_, w| unsafe {
        w.item0_unit().adc1();
        w.item0_channel().bits(7);
        w.item0_atten().db12();
        w.item1_unit().adc2();
        w.item1_channel().bits(1);
        w.item1_atten().db12()
    });
    apb_saradc
        .ctrl()
        .modify(|_, w| unsafe { w.sar_patt_len().bits(1) });

    pcr.sar_clk_div()
        .modify(|_, w| unsafe { w.sar1_clk_div_num().bits(15) });

    apb_saradc.ctrl2().modify(|_, w| unsafe {
        w.timer_target().bits(200);
        w.timer_en().set_bit()
    });

    rng_ll_enable();
}

/// Disables true randomness. Unlocks `ADC` peripheral.
// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32c61.c#L71-L88>
pub(crate) fn revert_trng() {
    let apb_saradc = APB_SARADC::regs();

    // `rng_ll_disable` is skipped: `Rng` keeps using the RNG.

    apb_saradc.ctrl2().modify(|_, w| w.timer_en().clear_bit());

    apb_saradc.sar_patt_tab1().reset();
    apb_saradc.sar_patt_tab2().reset();

    regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(0);
    regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(0);
    regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(0);
    regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(0);

    regi2c::ADC_SAR_DTEST_RTC.write_field(0);
    regi2c::ADC_SAR_ENT_RTC.write_field(0);
    regi2c::ADC_SAR1_ENCAL_REF.write_field(0);
    regi2c::ADC_SAR2_ENCAL_REF.write_field(0);

    // `regi2c_saradc_disable` is skipped: PERIF_I2C stays powered as set up by
    // `rtc_cntl::rtc::init`.

    PCR::regs().saradc_clkm_conf().modify(|_, w| unsafe {
        w.saradc_clkm_div_num().bits(4);
        w.saradc_clkm_div_b().bits(0);
        w.saradc_clkm_div_a().bits(0);
        w.saradc_clkm_sel().xtal()
    });
}

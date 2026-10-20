use crate::{
    peripherals::{APB_SARADC, HP_SYS_CLKRST, LP_ADC, PMU, RNG},
    soc::regi2c,
};

// `I2C_SAR_ADC_INIT_CODE_VAL`, see <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32p4.c#L17>
const SAR_ADC_INIT_CODE: u16 = 2166;

// `RNG_LL_CFG_PSCALE`, see <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/hal/esp32p4/include/hal/rng_ll.h#L18>
const RNG_TIMER_PSCALE: u8 = 255;

// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/hal/esp32p4/include/hal/rng_ll.h#L123-L132>
pub(crate) fn rng_ll_enable() {
    let rng = RNG::regs();
    rng.date().modify(|_, w| w.clk_en().set_bit());
    rng.rstn().modify(|_, w| w.rstn().clear_bit());
    rng.rstn().modify(|_, w| w.rstn().set_bit());
    rng.cfg()
        .modify(|_, w| unsafe { w.timer_pscale().bits(RNG_TIMER_PSCALE) });
    rng.cfg().modify(|_, w| w.timer_en().set_bit());
    rng.cfg().modify(|_, w| w.sample_enable().set_bit());
}

/// Enables true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32p4.c#L22-L65>
pub(crate) fn ensure_randomness() {
    let clkrst = HP_SYS_CLKRST::regs();
    let apb_saradc = APB_SARADC::regs();
    let lp_adc = LP_ADC::regs();

    clkrst.hp_rst_en2().modify(|_, w| w.rst_en_adc().set_bit());
    clkrst
        .hp_rst_en2()
        .modify(|_, w| w.rst_en_adc().clear_bit());

    clkrst
        .soc_clk_ctrl2()
        .modify(|_, w| w.adc_apb_clk_en().set_bit());
    clkrst
        .peri_clk_ctrl23()
        .modify(|_, w| w.adc_clk_en().set_bit());

    clkrst
        .peri_clk_ctrl22()
        .modify(|_, w| w.adc_clk_src_sel().xtal());
    clkrst.peri_clk_ctrl23().modify(|_, w| unsafe {
        w.adc_clk_div_num().bits(0);
        w.adc_clk_div_numerator().bits(0);
        w.adc_clk_div_denominator().bits(0)
    });

    // `regi2c_ctrl_ll_i2c_sar_periph_enable`, see <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/esp_hal_regi2c/esp32p4/include/hal/regi2c_ctrl_ll.h#L71-L80>
    let pmu = PMU::regs();
    pmu.rf_pwc().modify(|_, w| w.perif_i2c_rstb().clear_bit());
    crate::rom::ets_delay_us(1);
    pmu.rf_pwc().modify(|_, w| w.xpd_perif_i2c().set_bit());
    pmu.rf_pwc().modify(|_, w| w.perif_i2c_rstb().set_bit());

    regi2c::I2C_SAR_ADC_DTEST_VDD_GRP1.write_field(0);
    regi2c::I2C_SAR_ADC_ENT_VDD_GRP1.write_field(1);

    let [init_code_high, init_code_low] = SAR_ADC_INIT_CODE.to_be_bytes();
    regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(init_code_high);
    regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(init_code_low);

    apb_saradc.sar1_patt_tab1().modify(|_, w| unsafe {
        w.item0_channel().bits(10);
        w.item0_atten().db12();
        w.item1_channel().bits(10);
        w.item1_atten().db12();
        w.item2_channel().bits(10);
        w.item2_atten().db12();
        w.item3_channel().bits(10);
        w.item3_atten().db12()
    });
    apb_saradc
        .ctrl()
        .modify(|_, w| unsafe { w.sar1_patt_len().bits(0) });

    lp_adc
        .meas1_mux()
        .modify(|_, w| w.sar1_dig_force().set_bit());
    lp_adc.meas1_ctrl2().modify(|_, w| {
        w.meas1_start_force().set_bit();
        w.sar1_en_pad_force().set_bit()
    });

    apb_saradc.ctrl().modify(|_, w| {
        w.sar_clk_gated().set_bit();
        w.xpd_sar1_force().pu()
    });

    apb_saradc
        .ctrl()
        .modify(|_, w| unsafe { w.sar_clk_div().bits(15) });

    apb_saradc.ctrl2().modify(|_, w| unsafe {
        w.timer_target().bits(100);
        w.timer_sel().set_bit();
        w.timer_en().set_bit()
    });

    rng_ll_enable();
}

/// Disables true randomness. Unlocks `ADC` peripheral.
// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32p4.c#L67-L87>
pub(crate) fn revert_trng() {
    let clkrst = HP_SYS_CLKRST::regs();
    let apb_saradc = APB_SARADC::regs();
    let lp_adc = LP_ADC::regs();

    // `rng_ll_disable` is skipped: `Rng` keeps using the RNG.

    apb_saradc.ctrl2().modify(|_, w| w.timer_en().clear_bit());

    unsafe {
        apb_saradc.sar1_patt_tab1().write(|w| w.bits(0xFF_FFFF));
        apb_saradc.sar1_patt_tab2().write(|w| w.bits(0xFF_FFFF));
        apb_saradc.sar1_patt_tab3().write(|w| w.bits(0xFF_FFFF));
        apb_saradc.sar1_patt_tab4().write(|w| w.bits(0xFF_FFFF));
        apb_saradc.sar2_patt_tab1().write(|w| w.bits(0xFF_FFFF));
        apb_saradc.sar2_patt_tab2().write(|w| w.bits(0xFF_FFFF));
        apb_saradc.sar2_patt_tab3().write(|w| w.bits(0xFF_FFFF));
        apb_saradc.sar2_patt_tab4().write(|w| w.bits(0xFF_FFFF));
    }

    regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(0);
    regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(0);
    regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(0);
    regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(0);

    regi2c::I2C_SAR_ADC_DTEST_VDD_GRP1.write_field(0);
    regi2c::I2C_SAR_ADC_ENT_VDD_GRP1.write_field(0);

    // `regi2c_saradc_disable` is skipped: the ADC driver doesn't power PERIF_I2C itself.

    clkrst.peri_clk_ctrl23().modify(|_, w| unsafe {
        w.adc_clk_div_num().bits(4);
        w.adc_clk_div_numerator().bits(0);
        w.adc_clk_div_denominator().bits(0)
    });
    clkrst
        .peri_clk_ctrl22()
        .modify(|_, w| w.adc_clk_src_sel().xtal());

    lp_adc
        .meas1_mux()
        .modify(|_, w| w.sar1_dig_force().clear_bit());
    lp_adc.meas1_ctrl2().modify(|_, w| {
        w.meas1_start_force().set_bit();
        w.sar1_en_pad_force().set_bit()
    });
}

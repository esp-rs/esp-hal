use crate::{
    peripherals::{APB_SARADC, LP_PERI, PCR, PMU},
    soc::regi2c,
};

// `I2C_SAR_ADC_INIT_CODE_VAL`, see <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32h2.c#L16>
const SAR_ADC_INIT_CODE: u16 = 2150;

// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/hal/esp32h2/include/hal/rng_ll.h#L25-L28>
pub(crate) fn rng_ll_enable() {
    LP_PERI::regs()
        .clk_en()
        .modify(|_, w| w.rng_ck_en().set_bit());
}

/// Enables true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32h2.c#L21-L67>
pub(crate) fn ensure_randomness() {
    let pcr = PCR::regs();
    let pmu = PMU::regs();
    let apb_saradc = APB_SARADC::regs();

    unsafe {
        // Pull SAR ADC out of reset
        pcr.saradc_conf().modify(|_, w| w.saradc_rst_en().set_bit());

        pcr.saradc_conf()
            .modify(|_, w| w.saradc_rst_en().clear_bit());

        // Enable SAR ADC APB clock
        pcr.saradc_conf()
            .modify(|_, w| w.saradc_reg_clk_en().set_bit());

        // Enable ADC_CTRL_CLK (SAR ADC function clock)
        pcr.saradc_clkm_conf()
            .modify(|_, w| w.saradc_clkm_en().set_bit());

        // Select XTAL clock (40 MHz) source for ADC_CTRL_CLK
        pcr.saradc_clkm_conf()
            .modify(|_, w| w.saradc_clkm_sel().xtal());
        apb_saradc.ctrl().modify(|_, w| w.sar_clk_gated().set_bit());

        // Set the clock divider for ADC_CTRL_CLK to default value (in case it has been
        // changed)
        pcr.saradc_clkm_conf().modify(|_, w| {
            w.saradc_clkm_div_num().bits(0);
            w.saradc_clkm_div_b().bits(0);
            w.saradc_clkm_div_a().bits(0)
        });

        // some ADC sensor registers are in power group PERIF_I2C and need to be enabled
        // via PMU
        pmu.rf_pwc().modify(|_, w| w.xpd_perif_i2c().set_bit());

        // Config ADC circuit (Analog part) with I2C(HOST ID 0x69) and chose internal
        // voltage as sampling source
        regi2c::ADC_SARADC_DTEST.write_field(0);
        regi2c::ADC_SARADC_ENT_SAR.write_field(1);
        regi2c::ADC_SARADC_EN_TOUT_SAR1_BUS.write_field(1);

        let [init_code_high, init_code_low] = SAR_ADC_INIT_CODE.to_be_bytes();
        regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(init_code_high);
        regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(init_code_low);
        regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(init_code_high);
        regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(init_code_low);

        apb_saradc.sar_patt_tab1().modify(|_, w| {
            w.item0_unit().adc1();
            w.item0_channel().bits(0);
            w.item0_atten().db2_5();
            w.item1_unit().adc2();
            w.item1_channel().bits(1);
            w.item1_atten().db2_5()
        });

        // set pattern length to 1 (APB_SARADC_SAR_PATT_LEN counts from 0)
        apb_saradc.ctrl().modify(|_, w| w.sar_patt_len().bits(0));

        pcr.sar_clk_div()
            .modify(|_, w| w.sar1_clk_div_num().bits(15));

        // set timer expiry (timer is ADC_CTRL_CLK)
        apb_saradc.ctrl2().modify(|_, w| w.timer_target().bits(200));

        // enable timer
        apb_saradc.ctrl2().modify(|_, w| w.timer_en().set_bit());
    }

    rng_ll_enable();
}

/// Disables true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    APB_SARADC::regs()
        .ctrl2()
        .modify(|_, w| w.timer_en().clear_bit());

    APB_SARADC::regs().sar_patt_tab1().reset();

    // Revert ADC I2C configuration and initial voltage source setting
    regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(0);
    regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(0);
    regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(0);
    regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(0);
    regi2c::ADC_SARADC_DTEST.write_field(0);
    regi2c::ADC_SARADC_ENT_SAR.write_field(0);
    regi2c::ADC_SARADC_EN_TOUT_SAR1_BUS.write_field(0);
    // disable ADC_CTRL_CLK (SAR ADC function clock)
    PCR::regs().saradc_clkm_conf().reset();

    // Set PCR_SARADC_CONF_REG to initial state
    PCR::regs().saradc_conf().reset();
}

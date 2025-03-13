use crate::{
    peripherals::{APB_SARADC, PCR, PMU},
    soc::regi2c,
};

const SAR2_CHANNEL: u32 = 9;
const SAR2_ATTEN: u32 = 1;
const SAR1_ATTEN: u32 = 1;
const PATTERN_BIT_WIDTH: u32 = 6;

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
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
            .modify(|_, w| w.saradc_clkm_sel().bits(0));

        // Set the clock divider for ADC_CTRL_CLK to default value (in case it has been
        // changed)
        pcr.saradc_clkm_conf()
            .modify(|_, w| w.saradc_clkm_div_num().bits(0));

        // some ADC sensor registers are in power group PERIF_I2C and need to be enabled
        // via PMU
        pmu.rf_pwc().modify(|_, w| w.xpd_perif_i2c().set_bit());

        // Config ADC circuit (Analog part) with I2C(HOST ID 0x69) and chose internal
        // voltage as sampling source
        regi2c::ADC_SARADC_DTEST.write_field(2);
        regi2c::ADC_SARADC_ENT_SAR.write_field(1);
        regi2c::ADC_SARADC_EN_TOUT_SAR1_BUS.write_field(1);

        regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(0x08);
        regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(0x66);
        regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(0x08);
        regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(0x66);

        // create patterns and set them in pattern table
        let pattern_one: u32 = (SAR2_CHANNEL << 2) | SAR2_ATTEN; // we want channel 9 with max attenuation
        let pattern_two: u32 = SAR1_ATTEN; // we want channel 0 with max attenuation, channel doesn't really matter here
        let pattern_table: u32 =
            (pattern_two << (3 * PATTERN_BIT_WIDTH)) | (pattern_one << (2 * PATTERN_BIT_WIDTH));

        apb_saradc
            .sar_patt_tab1()
            .modify(|_, w| w.bits(pattern_table));

        // set pattern length to 2 (APB_SARADC_SAR_PATT_LEN counts from 0)
        apb_saradc.ctrl().modify(|_, w| w.sar_patt_len().bits(0));

        // Same as in C3
        apb_saradc.ctrl().modify(|_, w| w.sar_clk_div().bits(15));

        // set timer expiry (timer is ADC_CTRL_CLK)
        apb_saradc.ctrl2().modify(|_, w| w.timer_target().bits(200));

        // enable timer
        apb_saradc.ctrl2().modify(|_, w| w.timer_en().set_bit());
    }
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    unsafe {
        APB_SARADC::regs()
            .ctrl2()
            .modify(|_, w| w.timer_en().clear_bit());

        APB_SARADC::regs()
            .sar_patt_tab1()
            .modify(|_, w| w.bits(0xFFFFFF));

        // Revert ADC I2C configuration and initial voltage source setting
        regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(0x60);
        regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(0);
        regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(0x60);
        regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(0);
        regi2c::ADC_SARADC_DTEST.write_field(0);
        regi2c::ADC_SARADC_ENT_SAR.write_field(0);
        regi2c::ADC_SARADC_EN_TOUT_SAR1_BUS.write_field(0);
        // disable ADC_CTRL_CLK (SAR ADC function clock)
        PCR::regs()
            .saradc_clkm_conf()
            .modify(|_, w| w.bits(0x00404000));

        // Set PCR_SARADC_CONF_REG to initial state
        PCR::regs().saradc_conf().modify(|_, w| w.bits(0x5));
    }
}

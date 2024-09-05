const I2C_SAR_ADC: u8 = 0x69;
const I2C_SAR_ADC_HOSTID: u8 = 0;

const I2C_SARADC_DTEST: u8 = 7;
const I2C_SARADC_DTEST_MSB: u8 = 1;
const I2C_SARADC_DTEST_LSB: u8 = 0;
const I2C_SARADC_ENT_SAR: u8 = 7;
const I2C_SARADC_ENT_SAR_MSB: u8 = 3;
const I2C_SARADC_ENT_SAR_LSB: u8 = 1;
const I2C_SARADC_EN_TOUT_SAR1_BUS: u8 = 7;
const I2C_SARADC_EN_TOUT_SAR1_BUS_MSB: u8 = 5;
const I2C_SARADC_EN_TOUT_SAR1_BUS_LSB: u8 = 5;

const I2C_SARADC_SAR2_INIT_CODE_MSB: u8 = 4;
const I2C_SARADC_SAR2_INIT_CODE_MSB_MSB: u8 = 3;
const I2C_SARADC_SAR2_INIT_CODE_MSB_LSB: u8 = 0;
const I2C_SARADC_SAR2_INIT_CODE_LSB: u8 = 3;
const I2C_SARADC_SAR2_INIT_CODE_LSB_MSB: u8 = 7;
const I2C_SARADC_SAR2_INIT_CODE_LSB_LSB: u8 = 0;

const I2C_SARADC_SAR1_INIT_CODE_MSB: u8 = 1;
const I2C_SARADC_SAR1_INIT_CODE_MSB_MSB: u8 = 3;
const I2C_SARADC_SAR1_INIT_CODE_MSB_LSB: u8 = 0;
const I2C_SARADC_SAR1_INIT_CODE_LSB: u8 = 3;
const I2C_SARADC_SAR1_INIT_CODE_LSB_MSB: u8 = 7;
const I2C_SARADC_SAR1_INIT_CODE_LSB_LSB: u8 = 0;

const SAR2_CHANNEL: u32 = 9;
const SAR2_ATTEN: u32 = 1;
const SAR1_ATTEN: u32 = 1;
const PATTERN_BIT_WIDTH: u32 = 6;

const REGI2C_BBPLL: u8 = 0x66;
const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_PMU: u8 = 0x6d;
const REGI2C_ULP_CAL: u8 = 0x61;
const REGI2C_SAR_I2C: u8 = 0x69;

const I2C_MST_ANA_CONF1_M: u32 = 0x00FFFFFF;
const I2C_MST_I2C0_CTRL_REG: u32 = 0x600AD800;
const I2C_MST_ANA_CONF1_REG: u32 = I2C_MST_I2C0_CTRL_REG + 0x1c;

const REGI2C_BBPLL_RD_MASK: u32 = !(1 << 7) & I2C_MST_ANA_CONF1_M;
const REGI2C_BIAS_RD_MASK: u32 = !(1 << 6) & I2C_MST_ANA_CONF1_M;
const REGI2C_DIG_REG_RD_MASK: u32 = !(1 << 10) & I2C_MST_ANA_CONF1_M;
const REGI2C_ULP_CAL_RD_MASK: u32 = !(1 << 8) & I2C_MST_ANA_CONF1_M;
const REGI2C_SAR_I2C_RD_MASK: u32 = !(1 << 9) & I2C_MST_ANA_CONF1_M;
const REGI2C_RTC_BUSY: u32 = 1 << 25;

const REGI2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const REGI2C_RTC_SLAVE_ID_S: u8 = 0;
const REGI2C_RTC_ADDR_V: u8 = 0xFF;
const REGI2C_RTC_ADDR_S: u8 = 8;
const REGI2C_RTC_WR_CNTL_V: u8 = 0x1;
const REGI2C_RTC_WR_CNTL_S: u8 = 24;
const REGI2C_RTC_DATA_V: u8 = 0xFF;
const REGI2C_RTC_DATA_S: u8 = 16;

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    let pcr = unsafe { &*crate::peripherals::PCR::ptr() };
    let pmu = unsafe { &*crate::peripherals::PMU::ptr() };
    let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::ptr() };

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
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_DTEST,
            I2C_SARADC_DTEST_MSB,
            I2C_SARADC_DTEST_LSB,
            2,
        );
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_ENT_SAR,
            I2C_SARADC_ENT_SAR_MSB,
            I2C_SARADC_ENT_SAR_LSB,
            1,
        );
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_EN_TOUT_SAR1_BUS,
            I2C_SARADC_EN_TOUT_SAR1_BUS_MSB,
            I2C_SARADC_EN_TOUT_SAR1_BUS_LSB,
            1,
        );

        // SAR2 High ADDR
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR2_INIT_CODE_MSB,
            I2C_SARADC_SAR2_INIT_CODE_MSB_MSB,
            I2C_SARADC_SAR2_INIT_CODE_MSB_LSB,
            0x08,
        );
        // SAR2 Low ADDR
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR2_INIT_CODE_LSB,
            I2C_SARADC_SAR2_INIT_CODE_LSB_MSB,
            I2C_SARADC_SAR2_INIT_CODE_LSB_LSB,
            0x66,
        );
        // SAR1 High ADDR
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR1_INIT_CODE_MSB,
            I2C_SARADC_SAR1_INIT_CODE_MSB_MSB,
            I2C_SARADC_SAR1_INIT_CODE_MSB_LSB,
            0x08,
        );
        // SAR1 Low ADDR
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR1_INIT_CODE_LSB,
            I2C_SARADC_SAR1_INIT_CODE_LSB_MSB,
            I2C_SARADC_SAR1_INIT_CODE_LSB_LSB,
            0x66,
        );

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
    let apb_saradc = unsafe { &*crate::peripherals::APB_SARADC::ptr() };
    let pcr = unsafe { &*crate::peripherals::PCR::ptr() };

    unsafe {
        apb_saradc.ctrl2().modify(|_, w| w.timer_en().clear_bit());

        apb_saradc.sar_patt_tab1().modify(|_, w| w.bits(0xFFFFFF));

        // Revert ADC I2C configuration and initial voltage source setting
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR2_INIT_CODE_MSB,
            I2C_SARADC_SAR2_INIT_CODE_MSB_MSB,
            I2C_SARADC_SAR2_INIT_CODE_MSB_LSB,
            0x60,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR2_INIT_CODE_LSB,
            I2C_SARADC_SAR2_INIT_CODE_LSB_MSB,
            I2C_SARADC_SAR2_INIT_CODE_LSB_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR1_INIT_CODE_MSB,
            I2C_SARADC_SAR1_INIT_CODE_MSB_MSB,
            I2C_SARADC_SAR1_INIT_CODE_MSB_LSB,
            0x60,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_SAR1_INIT_CODE_LSB,
            I2C_SARADC_SAR1_INIT_CODE_LSB_MSB,
            I2C_SARADC_SAR1_INIT_CODE_LSB_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_DTEST,
            I2C_SARADC_DTEST_MSB,
            I2C_SARADC_DTEST_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_ENT_SAR,
            I2C_SARADC_ENT_SAR_MSB,
            I2C_SARADC_ENT_SAR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            I2C_SARADC_EN_TOUT_SAR1_BUS,
            I2C_SARADC_EN_TOUT_SAR1_BUS_MSB,
            I2C_SARADC_EN_TOUT_SAR1_BUS_LSB,
            0,
        );

        // disable ADC_CTRL_CLK (SAR ADC function clock)
        pcr.saradc_clkm_conf().modify(|_, w| w.bits(0x00404000));

        // Set PCR_SARADC_CONF_REG to initial state
        pcr.saradc_conf().modify(|_, w| w.bits(0x5));
    }
}

fn regi2c_enable_block(block: u8) {
    let modem_lpcon = unsafe { &*crate::peripherals::MODEM_LPCON::ptr() };

    modem_lpcon
        .clk_conf()
        .modify(|_, w| w.clk_i2c_mst_en().set_bit());

    // Before config I2C register, enable corresponding slave.
    match block {
        v if v == REGI2C_BBPLL => {
            reg_set_bit(I2C_MST_ANA_CONF1_REG, REGI2C_BBPLL_RD_MASK);
        }
        v if v == REGI2C_BIAS => {
            reg_set_bit(I2C_MST_ANA_CONF1_REG, REGI2C_BIAS_RD_MASK);
        }
        v if v == REGI2C_PMU => {
            reg_set_bit(I2C_MST_ANA_CONF1_REG, REGI2C_DIG_REG_RD_MASK);
        }
        v if v == REGI2C_ULP_CAL => {
            reg_set_bit(I2C_MST_ANA_CONF1_REG, REGI2C_ULP_CAL_RD_MASK);
        }
        v if v == REGI2C_SAR_I2C => {
            reg_set_bit(I2C_MST_ANA_CONF1_REG, REGI2C_SAR_I2C_RD_MASK);
        }
        _ => (),
    }
}

fn regi2c_disable_block(block: u8) {
    match block {
        v if v == REGI2C_BBPLL => {
            reg_clr_bit(I2C_MST_ANA_CONF1_REG, REGI2C_BBPLL_RD_MASK);
        }
        v if v == REGI2C_BIAS => {
            reg_clr_bit(I2C_MST_ANA_CONF1_REG, REGI2C_BIAS_RD_MASK);
        }
        v if v == REGI2C_PMU => {
            reg_clr_bit(I2C_MST_ANA_CONF1_REG, REGI2C_DIG_REG_RD_MASK);
        }
        v if v == REGI2C_ULP_CAL => {
            reg_clr_bit(I2C_MST_ANA_CONF1_REG, REGI2C_ULP_CAL_RD_MASK);
        }
        v if v == REGI2C_SAR_I2C => {
            reg_clr_bit(I2C_MST_ANA_CONF1_REG, REGI2C_SAR_I2C_RD_MASK);
        }
        _ => (),
    }
}

fn reg_write(reg: u32, v: u32) {
    unsafe {
        (reg as *mut u32).write_volatile(v);
    }
}

fn reg_get_bit(reg: u32, b: u32) -> u32 {
    unsafe { (reg as *mut u32).read_volatile() & b }
}

fn reg_set_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | bit);
    }
}

fn reg_clr_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !bit);
    }
}

fn reg_get_field(reg: u32, s: u32, v: u32) -> u32 {
    unsafe { ((reg as *mut u32).read_volatile() >> s) & v }
}

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb - lsb < 8);
    regi2c_enable_block(block);

    // Read the i2c bus register
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}

    let mut temp: u32 = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32)
        << REGI2C_RTC_SLAVE_ID_S as u32)
        | (reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32;
    reg_write(I2C_MST_I2C0_CTRL_REG, temp);
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}
    temp = reg_get_field(
        I2C_MST_I2C0_CTRL_REG,
        REGI2C_RTC_DATA_S as u32,
        REGI2C_RTC_DATA_V as u32,
    );
    // Write the i2c bus register
    temp &= (!(0xFFFFFFFF << lsb)) | (0xFFFFFFFF << (msb + 1));
    temp |= (data as u32 & (!(0xFFFFFFFF << (msb as u32 - lsb as u32 + 1)))) << (lsb as u32);
    temp = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S as u32)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32)
        | ((0x1 & REGI2C_RTC_WR_CNTL_V as u32) << REGI2C_RTC_WR_CNTL_S as u32)
        | ((temp & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S as u32);
    reg_write(I2C_MST_I2C0_CTRL_REG, temp);
    while reg_get_bit(I2C_MST_I2C0_CTRL_REG, REGI2C_RTC_BUSY) != 0 {}

    regi2c_disable_block(block);
}

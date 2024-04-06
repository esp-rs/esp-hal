const PCR_SARADC_CONF_REG: u32 = 0x60096000 + 0x80;
const PCR_SARADC_RST_EN: u32 = 1 << 1;
const PCR_SARADC_REG_CLK_EN: u32 = 1 << 2;
const PCR_SARADC_CLKM_CONF_REG: u32 = 0x60096000 + 0x84;
const PCR_SARADC_CLKM_EN: u32 = 1 << 22;
const PCR_SARADC_CLKM_SEL_V: u32 = 0x00000003;
const PCR_SARADC_CLKM_SEL_S: u32 = 20;
const PCR_SARADC_CLKM_DIV_NUM_V: u32 = 0x000000FF;
const PCR_SARADC_CLKM_DIV_NUM_S: u32 = 12;
const PMU_RF_PWC_REG: u32 = 0x600B0000 + 0x154;
const PMU_XPD_PERIF_I2C: u32 = 1 << 27;
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
const APB_SARADC_SAR_PATT_TAB1_REG: u32 = 0x60040000 + 0x18;
const APB_SARADC_CTRL_REG: u32 = 0x60040000;
const APB_SARADC_CTRL2_REG: u32 = 0x60040000 + 0x4;

const APB_SARADC_SARADC_SAR_PATT_LEN_V: u32 = 0x00000007;
const APB_SARADC_SARADC_SAR_PATT_LEN_S: u32 = 15;
const APB_SARADC_SARADC_SAR_CLK_DIV_V: u32 = 0x000000FF;
const APB_SARADC_SARADC_SAR_CLK_DIV_S: u32 = 7;
const APB_SARADC_SARADC_TIMER_TARGET_V: u32 = 0x00000FFF;
const APB_SARADC_SARADC_TIMER_TARGET_S: u32 = 12;
const APB_SARADC_SARADC_TIMER_EN: u32 = 1 << 24;

const DR_REG_MODEM_LPCON_BASE: u32 = 0x600AF000;
const MODEM_LPCON_CLK_CONF_REG: u32 = DR_REG_MODEM_LPCON_BASE + 0x18;
const MODEM_LPCON_CLK_I2C_MST_EN: u32 = 1 << 2;
const DR_REG_LP_I2C_ANA_MST_BASE: u32 = 0x600B2400;
const LP_I2C_ANA_MST_DATE_REG: u32 = DR_REG_LP_I2C_ANA_MST_BASE + 0x3fc;
const LP_I2C_ANA_MST_I2C_MAT_CLK_EN: u32 = 1 << 28;

const REGI2C_BBPLL: u8 = 0x66;
const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_DIG_REG: u8 = 0x6d;
const REGI2C_ULP_CAL: u8 = 0x61;
const REGI2C_SAR_I2C: u8 = 0x69;

const LP_I2C_ANA_MST_DEVICE_EN_REG: u32 = DR_REG_LP_I2C_ANA_MST_BASE + 0x14;
const REGI2C_BBPLL_DEVICE_EN: u32 = 1 << 5;
const REGI2C_BIAS_DEVICE_EN: u32 = 1 << 4;
const REGI2C_DIG_REG_DEVICE_EN: u32 = 1 << 8;
const REGI2C_ULP_CAL_DEVICE_EN: u32 = 1 << 6;
const REGI2C_SAR_I2C_DEVICE_EN: u32 = 1 << 7;
const REGI2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const REGI2C_RTC_SLAVE_ID_S: u8 = 0;
const REGI2C_RTC_ADDR_V: u8 = 0xFF;
const REGI2C_RTC_ADDR_S: u8 = 8;
const REGI2C_RTC_WR_CNTL_V: u8 = 0x1;
const REGI2C_RTC_WR_CNTL_S: u8 = 24;
const REGI2C_RTC_DATA_V: u8 = 0xFF;
const REGI2C_RTC_DATA_S: u8 = 16;

const LP_I2C_ANA_MST_I2C0_CTRL_REG: u32 = DR_REG_LP_I2C_ANA_MST_BASE;
const LP_I2C_ANA_MST_I2C0_BUSY: u32 = 1 << 25;

const LP_I2C_ANA_MST_I2C0_DATA_REG: u32 = DR_REG_LP_I2C_ANA_MST_BASE + 0x8;
const LP_I2C_ANA_MST_I2C0_RDATA_V: u32 = 0x000000FF;
const LP_I2C_ANA_MST_I2C0_RDATA_S: u32 = 0;

pub(crate) fn ensure_randomness() {
    // Pull SAR ADC out of reset
    reg_set_bit(PCR_SARADC_CONF_REG, PCR_SARADC_RST_EN);
    reg_clr_bit(PCR_SARADC_CONF_REG, PCR_SARADC_RST_EN);

    // Enable SAR ADC APB clock
    reg_set_bit(PCR_SARADC_CONF_REG, PCR_SARADC_REG_CLK_EN);

    // Enable ADC_CTRL_CLK (SAR ADC function clock)
    reg_set_bit(PCR_SARADC_CLKM_CONF_REG, PCR_SARADC_CLKM_EN);

    // Select XTAL clock (40 MHz) source for ADC_CTRL_CLK
    reg_set_field(
        PCR_SARADC_CLKM_CONF_REG,
        PCR_SARADC_CLKM_SEL_V,
        PCR_SARADC_CLKM_SEL_S,
        0,
    );

    // Set the clock divider for ADC_CTRL_CLK to default value (in case it has been
    // changed)
    reg_set_field(
        PCR_SARADC_CLKM_CONF_REG,
        PCR_SARADC_CLKM_DIV_NUM_V,
        PCR_SARADC_CLKM_DIV_NUM_S,
        0,
    );

    // some ADC sensor registers are in power group PERIF_I2C and need to be enabled
    // via PMU
    set_peri_reg_mask(PMU_RF_PWC_REG, PMU_XPD_PERIF_I2C);

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
    reg_write(APB_SARADC_SAR_PATT_TAB1_REG, pattern_table);

    // set pattern length to 2 (APB_SARADC_SAR_PATT_LEN counts from 0)
    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SARADC_SAR_PATT_LEN_V,
        APB_SARADC_SARADC_SAR_PATT_LEN_S,
        0,
    );

    // Same as in C3
    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SARADC_SAR_CLK_DIV_V,
        APB_SARADC_SARADC_SAR_CLK_DIV_S,
        15,
    );

    // set timer expiry (timer is ADC_CTRL_CLK)
    reg_set_field(
        APB_SARADC_CTRL2_REG,
        APB_SARADC_SARADC_TIMER_TARGET_V,
        APB_SARADC_SARADC_TIMER_TARGET_S,
        200,
    );

    // enable timer
    reg_set_bit(APB_SARADC_CTRL2_REG, APB_SARADC_SARADC_TIMER_EN);
}

pub(crate) fn revert_trng() {
    // Disable timer
    reg_clr_bit(APB_SARADC_CTRL2_REG, APB_SARADC_SARADC_TIMER_EN);

    // Write reset value
    reg_write(APB_SARADC_SAR_PATT_TAB1_REG, 0xFFFFFF);

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
        I2C_SARADC_EN_TOUT_SAR1_BUS
        I2C_SARADC_EN_TOUT_SAR1_BUS_MSB,
        I2C_SARADC_EN_TOUT_SAR1_BUS_LSB,
        0,
    );

    // disable ADC_CTRL_CLK (SAR ADC function clock)
    reg_write(PCR_SARADC_CLKM_CONF_REG, 0x00404000);
    
    // Set PCR_SARADC_CONF_REG to initial state
    reg_write(PCR_SARADC_CONF_REG, 0x5);
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

fn reg_set_field(reg: u32, field_v: u32, field_s: u32, value: u32) {
    unsafe {
        (reg as *mut u32).write_volatile(
            ((reg as *mut u32).read_volatile() & !(field_v << field_s))
                | ((value & field_v) << field_s),
        )
    }
}

fn set_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
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

fn reg_get_field(reg: u32, s: u32, v: u32) -> u32 {
    unsafe { ((reg as *mut u32).read_volatile() >> s) & v }
}

fn regi2c_enable_block(block: u8) {
    reg_set_bit(MODEM_LPCON_CLK_CONF_REG, MODEM_LPCON_CLK_I2C_MST_EN);
    reg_set_bit(LP_I2C_ANA_MST_DATE_REG, LP_I2C_ANA_MST_I2C_MAT_CLK_EN);

    // Before config I2C register, enable corresponding slave.
    match block {
        v if v == REGI2C_BBPLL => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BBPLL_DEVICE_EN);
        }
        v if v == REGI2C_BIAS => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BIAS_DEVICE_EN);
        }
        v if v == REGI2C_DIG_REG => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_DIG_REG_DEVICE_EN);
        }
        v if v == REGI2C_ULP_CAL => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_ULP_CAL_DEVICE_EN);
        }
        v if v == REGI2C_SAR_I2C => {
            reg_set_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_SAR_I2C_DEVICE_EN);
        }
        _ => (),
    }
}

fn regi2c_disable_block(block: u8) {
    match block {
        v if v == REGI2C_BBPLL => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BBPLL_DEVICE_EN);
        }
        v if v == REGI2C_BIAS => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_BIAS_DEVICE_EN);
        }
        v if v == REGI2C_DIG_REG => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_DIG_REG_DEVICE_EN);
        }
        v if v == REGI2C_ULP_CAL => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_ULP_CAL_DEVICE_EN);
        }
        v if v == REGI2C_SAR_I2C => {
            reg_clr_bit(LP_I2C_ANA_MST_DEVICE_EN_REG, REGI2C_SAR_I2C_DEVICE_EN);
        }
        _ => (),
    }
}

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb - lsb < 8);
    regi2c_enable_block(block);

    // Read the i2c bus register
    let mut temp: u32 = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32)
        << REGI2C_RTC_SLAVE_ID_S as u32)
        | (reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32;
    reg_write(LP_I2C_ANA_MST_I2C0_CTRL_REG, temp);
    while reg_get_bit(LP_I2C_ANA_MST_I2C0_CTRL_REG, LP_I2C_ANA_MST_I2C0_BUSY) != 0 {}
    temp = reg_get_field(
        LP_I2C_ANA_MST_I2C0_DATA_REG,
        LP_I2C_ANA_MST_I2C0_RDATA_S,
        LP_I2C_ANA_MST_I2C0_RDATA_V,
    );
    // Write the i2c bus register
    temp &= (!(0xFFFFFFFF << lsb)) | (0xFFFFFFFF << (msb + 1));
    temp |= (data as u32 & (!(0xFFFFFFFF << (msb as u32 - lsb as u32 + 1)))) << (lsb as u32);
    temp = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S as u32)
        | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32)
        | ((0x1 & REGI2C_RTC_WR_CNTL_V as u32) << REGI2C_RTC_WR_CNTL_S as u32)
        | ((temp & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S as u32);
    reg_write(LP_I2C_ANA_MST_I2C0_CTRL_REG, temp);
    while reg_get_bit(LP_I2C_ANA_MST_I2C0_CTRL_REG, LP_I2C_ANA_MST_I2C0_BUSY) != 0 {}

    regi2c_disable_block(block);
}

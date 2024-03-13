const DR_REG_RTCCNTL_BASE: u32 = 0x3f408000;
const DR_REG_SYSTEM_BASE: u32 = 0x3f4c0000;
const DR_REG_SENS_BASE: u32 = 0x3f408800;
const DR_REG_APB_SARADC_BASE: u32 = 0x3f440000;
const DPORT_APB_SARADC_CLK_EN: u32 = 1 << 28;
const RTC_CNTL_CLK_CONF_REG: u32 = DR_REG_RTCCNTL_BASE + 0x0074;
const RTC_CNTL_DIG_CLK8M_EN: u32 = 1 << 10;
const DPORT_PERIP_CLK_EN0_REG: u32 = DR_REG_SYSTEM_BASE + 0x040;
const APB_SARADC_APB_ADC_CLKM_CONF_REG: u32 = DR_REG_APB_SARADC_BASE + 0x05c;
const APB_SARADC_CLK_SEL_V: u32 = 0x3;
const APB_SARADC_CLK_SEL_S: u32 = 21;
const RTC_CNTL_ANA_CONF_REG: u32 = DR_REG_RTCCNTL_BASE + 0x0034;
const RTC_CNTL_SAR_I2C_FORCE_PD_M: u32 = 1 << 21;
const RTC_CNTL_SAR_I2C_FORCE_PU_M: u32 = 1 << 22;
const ANA_CONFIG_REG: u32 = 0x6000E044;
const ANA_CONFIG2_REG: u32 = 0x6000E048;
const I2C_SAR_ADC: u8 = 0x69;
const I2C_SAR_ADC_HOSTID: u8 = 1;
const ADC_SAR1_DREF_ADDR: u8 = 0x2;
const ADC_SAR1_DREF_ADDR_MSB: u8 = 0x6;
const ADC_SAR1_DREF_ADDR_LSB: u8 = 0x4;
const ADC_SAR2_DREF_ADDR: u8 = 0x5;
const ADC_SAR2_DREF_ADDR_MSB: u8 = 0x6;
const ADC_SAR2_DREF_ADDR_LSB: u8 = 0x4;
const ADC_SARADC_ENCAL_REF_ADDR: u8 = 0x7;
const ADC_SARADC_ENCAL_REF_ADDR_MSB: u8 = 4;
const ADC_SARADC_ENCAL_REF_ADDR_LSB: u8 = 4;
const ADC_SARADC_ENT_TSENS_ADDR: u8 = 0x7;
const ADC_SARADC_ENT_TSENS_ADDR_MSB: u8 = 2;
const ADC_SARADC_ENT_TSENS_ADDR_LSB: u8 = 2;
const ADC_SARADC_ENT_RTC_ADDR: u8 = 0x7;
const ADC_SARADC_ENT_RTC_ADDR_MSB: u8 = 3;
const ADC_SARADC_ENT_RTC_ADDR_LSB: u8 = 3;
const APB_SARADC_CTRL_REG: u32 = DR_REG_APB_SARADC_BASE;
const APB_SARADC_SAR1_PATT_LEN_V: u32 = 0xF;
const APB_SARADC_SAR1_PATT_LEN_S: u32 = 15;
const APB_SARADC_SAR1_PATT_TAB1_REG: u32 = DR_REG_APB_SARADC_BASE + 0x018;
const APB_SARADC_SAR2_PATT_LEN_V: u32 = 0xF;
const APB_SARADC_SAR2_PATT_LEN_S: u32 = 19;
const APB_SARADC_SAR2_PATT_TAB1_REG: u32 = DR_REG_APB_SARADC_BASE + 0x028;
const SENS_SAR_MEAS1_MUX_REG: u32 = DR_REG_SENS_BASE + 0x0010;
const SENS_SAR1_DIG_FORCE: u32 = 1 << 31;
const APB_SARADC_WORK_MODE_V: u32 = 0x3;
const APB_SARADC_WORK_MODE_S: u32 = 3;

const APB_SARADC_CTRL2_REG: u32 = DR_REG_APB_SARADC_BASE + 0x004;
const APB_SARADC_MEAS_NUM_LIMIT: u32 = 1 << 0;
const SENS_SAR_POWER_XPD_SAR_REG: u32 = DR_REG_SENS_BASE + 0x003c;
const SENS_FORCE_XPD_SAR_V: u32 = 0x3;
const SENS_FORCE_XPD_SAR_S: u32 = 29;
const APB_SARADC_TIMER_SEL: u32 = 1 << 11;
const APB_SARADC_TIMER_TARGET_V: u32 = 0xFFF;
const APB_SARADC_TIMER_TARGET_S: u32 = 12;
const APB_SARADC_START_FORCE: u32 = 1 << 0;
const APB_SARADC_TIMER_EN: u32 = 1 << 24;

const I2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const I2C_RTC_SLAVE_ID_S: u8 = 0;
const I2C_RTC_ADDR_V: u8 = 0xFF;
const I2C_RTC_ADDR_S: u8 = 0x8;
const I2C_RTC_CONFIG2: u32 = 0x6000e000;
const I2C_RTC_BUSY: u32 = 1 << 25;
const I2C_RTC_DATA_V: u32 = 0xFF;
const I2C_RTC_DATA_S: u32 = 16;
const I2C_RTC_WR_CNTL_V: u8 = 0x1;
const I2C_RTC_WR_CNTL_S: u8 = 24;
const I2C_RTC_CONFIG0: u32 = 0x6000e048;
const I2C_RTC_CONFIG1: u32 = 0x6000e044;
const I2C_RTC_MAGIC_CTRL_V: u32 = 0x1FFF;
const I2C_RTC_MAGIC_CTRL_S: u32 = 4;
const I2C_RTC_MAGIC_DEFAULT: u32 = 0x1c40;
const I2C_RTC_ALL_MASK_V: u32 = 0x7FFF;
const I2C_RTC_ALL_MASK_S: u32 = 8;
const I2C_RTC_WIFI_CLK_EN: u32 = 0x3f426000 + 0x090;
const I2C_RTC_CLK_GATE_EN: u32 = 1 << 18;
const I2C_BOD: u8 = 0x61;
const I2C_BBPLL: u8 = 0x66;
const I2C_APLL: u8 = 0x6D;
const I2C_RTC_APLL_MASK: u32 = 1 << 14;
const I2C_RTC_BBPLL_MASK: u32 = 1 << 17;
const I2C_RTC_SAR_MASK: u32 = 1 << 18;
const I2C_RTC_BOD_MASK: u32 = 1 << 22;

pub(crate) fn ensure_randomness() {
    // Enable 8M clock source for RNG (this is actually enough to produce strong
    // random results, but enabling the SAR ADC as well adds some insurance.)
    reg_set_bit(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_CLK8M_EN);

    // Enable SAR ADC to read a disconnected input for additional entropy
    set_peri_reg_mask(DPORT_PERIP_CLK_EN0_REG, DPORT_APB_SARADC_CLK_EN);

    reg_set_field(
        APB_SARADC_APB_ADC_CLKM_CONF_REG,
        APB_SARADC_CLK_SEL_V,
        APB_SARADC_CLK_SEL_S,
        2,
    );

    clear_peri_reg_mask(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_SAR_I2C_FORCE_PD_M);
    set_peri_reg_mask(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_SAR_I2C_FORCE_PU_M);
    clear_peri_reg_mask(ANA_CONFIG_REG, 1 << 18);
    set_peri_reg_mask(ANA_CONFIG2_REG, 1 << 16);

    regi2c_write_mask(
        I2C_SAR_ADC,
        I2C_SAR_ADC_HOSTID,
        ADC_SAR1_DREF_ADDR,
        ADC_SAR1_DREF_ADDR_MSB,
        ADC_SAR1_DREF_ADDR_LSB,
        0x4,
    );

    regi2c_write_mask(
        I2C_SAR_ADC,
        I2C_SAR_ADC_HOSTID,
        ADC_SAR2_DREF_ADDR,
        ADC_SAR2_DREF_ADDR_MSB,
        ADC_SAR2_DREF_ADDR_LSB,
        0x4,
    );

    regi2c_write_mask(
        I2C_SAR_ADC,
        I2C_SAR_ADC_HOSTID,
        ADC_SARADC_ENCAL_REF_ADDR,
        ADC_SARADC_ENCAL_REF_ADDR_MSB,
        ADC_SARADC_ENCAL_REF_ADDR_LSB,
        1,
    );

    regi2c_write_mask(
        I2C_SAR_ADC,
        I2C_SAR_ADC_HOSTID,
        ADC_SARADC_ENT_TSENS_ADDR,
        ADC_SARADC_ENT_TSENS_ADDR_MSB,
        ADC_SARADC_ENT_TSENS_ADDR_LSB,
        1,
    );

    regi2c_write_mask(
        I2C_SAR_ADC,
        I2C_SAR_ADC_HOSTID,
        ADC_SARADC_ENT_RTC_ADDR,
        ADC_SARADC_ENT_RTC_ADDR_MSB,
        ADC_SARADC_ENT_RTC_ADDR_LSB,
        0,
    );

    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SAR1_PATT_LEN_V,
        APB_SARADC_SAR1_PATT_LEN_S,
        0,
    );

    write_peri_reg(APB_SARADC_SAR1_PATT_TAB1_REG, 0xafffffff);

    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SAR2_PATT_LEN_V,
        APB_SARADC_SAR2_PATT_LEN_S,
        0,
    );

    write_peri_reg(APB_SARADC_SAR2_PATT_TAB1_REG, 0xafffffff);

    set_peri_reg_mask(SENS_SAR_MEAS1_MUX_REG, SENS_SAR1_DIG_FORCE);

    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_WORK_MODE_V,
        APB_SARADC_WORK_MODE_S,
        1,
    );

    clear_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_MEAS_NUM_LIMIT);

    reg_set_field(
        SENS_SAR_POWER_XPD_SAR_REG,
        SENS_FORCE_XPD_SAR_V,
        SENS_FORCE_XPD_SAR_S,
        3,
    );

    set_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_SEL);

    reg_set_field(
        APB_SARADC_CTRL2_REG,
        APB_SARADC_TIMER_TARGET_V,
        APB_SARADC_TIMER_TARGET_S,
        100,
    );

    clear_peri_reg_mask(APB_SARADC_CTRL_REG, APB_SARADC_START_FORCE);
    set_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_EN);
}

fn reg_set_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | bit);
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

fn reg_clr_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !bit);
    }
}

fn regi2c_enable_block(block: u8) {
    reg_set_field(
        I2C_RTC_CONFIG0,
        I2C_RTC_MAGIC_CTRL_V,
        I2C_RTC_MAGIC_CTRL_S,
        I2C_RTC_MAGIC_DEFAULT,
    );
    reg_set_field(
        I2C_RTC_CONFIG1,
        I2C_RTC_ALL_MASK_V,
        I2C_RTC_ALL_MASK_S,
        I2C_RTC_ALL_MASK_V,
    );

    reg_set_bit(I2C_RTC_WIFI_CLK_EN, I2C_RTC_CLK_GATE_EN);

    // Before config I2C register, enable corresponding slave.
    match block {
        v if v == I2C_APLL => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_APLL_MASK);
        }
        v if v == I2C_BBPLL => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_BBPLL_MASK);
        }
        v if v == I2C_SAR_ADC => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_SAR_MASK);
        }
        v if v == I2C_BOD => {
            reg_set_bit(I2C_RTC_CONFIG1, I2C_RTC_BOD_MASK);
        }
        _ => (),
    }
}

fn regi2c_disable_block(block: u8) {
    match block {
        v if v == I2C_APLL => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_APLL_MASK);
        }
        v if v == I2C_BBPLL => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_BBPLL_MASK);
        }
        v if v == I2C_SAR_ADC => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_SAR_MASK);
        }
        v if v == I2C_BOD => {
            reg_clr_bit(I2C_RTC_CONFIG1, I2C_RTC_BOD_MASK);
        }
        _ => (),
    }
}

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb - lsb < 8);
    regi2c_enable_block(block);

    // Read the i2c bus register
    let mut temp: u32 = ((block as u32 & I2C_RTC_SLAVE_ID_V as u32) << I2C_RTC_SLAVE_ID_S as u32)
        | (reg_add as u32 & I2C_RTC_ADDR_V as u32) << I2C_RTC_ADDR_S as u32;
    reg_write(I2C_RTC_CONFIG2, temp);
    while reg_get_bit(I2C_RTC_CONFIG2, I2C_RTC_BUSY) != 0 {}
    // Write the i2c bus register
    temp = (data as u32 & (!(0xFFFFFFFF << (msb as u32 - lsb as u32 + 1)))) << (lsb as u32);
    temp = ((block as u32 & I2C_RTC_SLAVE_ID_V as u32) << I2C_RTC_SLAVE_ID_S as u32)
        | ((reg_add as u32 & I2C_RTC_ADDR_V as u32) << I2C_RTC_ADDR_S as u32)
        | ((0x1 & I2C_RTC_WR_CNTL_V as u32) << I2C_RTC_WR_CNTL_S as u32)
        | ((temp & I2C_RTC_DATA_V) << I2C_RTC_DATA_S);
    reg_write(I2C_RTC_CONFIG2, temp);
    while reg_get_bit(I2C_RTC_CONFIG2, I2C_RTC_BUSY) != 0 {}

    regi2c_disable_block(block);
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

fn clear_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
    }
}

fn write_peri_reg(reg: u32, val: u32) {
    unsafe {
        (reg as *mut u32).write_volatile(val);
    }
}

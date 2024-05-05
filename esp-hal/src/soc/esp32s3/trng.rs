const DR_REG_SYSCON_BASE: u32 = 0x60026000;
const DR_REG_RTCCNTL_BASE: u32 = 0x60008000;
const DR_REG_SYSTEM_BASE: u32 = 0x600C0000;
const DR_REG_APB_SARADC_BASE: u32 = 0x60040000;
const DR_REG_SENS_BASE: u32 = 0x60008800;
const SYSTEM_WIFI_CLK_EN_REG: u32 = DR_REG_SYSCON_BASE + 0x14;
const SYSTEM_WIFI_CLK_RNG_EN: u32 = 1 << 15;
const RTC_CNTL_CLK_CONF_REG: u32 = DR_REG_RTCCNTL_BASE + 0x74;
const RTC_CNTL_DIG_CLK8M_EN: u32 = 1 << 10;
const SYSTEM_PERIP_CLK_EN0_REG: u32 = DR_REG_SYSTEM_BASE + 0x18;
const SYSTEM_APB_SARADC_CLK_EN: u32 = 1 << 28;
const APB_SARADC_APB_ADC_CLKM_CONF_REG: u32 = DR_REG_APB_SARADC_BASE + 0x70;
const APB_SARADC_CLK_SEL_V: u32 = 0x3;
const APB_SARADC_CLK_SEL_S: u32 = 21;
const APB_SARADC_CTRL_REG: u32 = DR_REG_APB_SARADC_BASE;
const APB_SARADC_CTRL2_REG: u32 = DR_REG_APB_SARADC_BASE + 0x4;
const APB_SARADC_SAR_CLK_GATED: u32 = 1 << 6;
const APB_SARADC_CLK_EN: u32 = 1 << 20;
const APB_SARADC_CLKM_DIV_NUM_V: u32 = 0xFF;
const APB_SARADC_CLKM_DIV_NUM_S: u32 = 0;
const APB_SARADC_SAR_CLK_DIV_V: u32 = 0xFF;
const APB_SARADC_SAR_CLK_DIV_S: u32 = 7;
const APB_SARADC_TIMER_TARGET_V: u32 = 0xFFF;
const APB_SARADC_TIMER_TARGET_S: u32 = 0x12;
const APB_SARADC_START_FORCE: u32 = 1 << 0;
const SENS_SAR_POWER_XPD_SAR_REG: u32 = DR_REG_SENS_BASE + 0x3C;
const SENS_FORCE_XPD_SAR_V: u32 = 0x3;
const SENS_FORCE_XPD_SAR_S: u32 = 29;
const APB_SARADC_MEAS_NUM_LIMIT: u32 = 1 << 0;
const APB_SARADC_WORK_MODE_V: u32 = 0x3;
const APB_SARADC_WORK_MODE_S: u32 = 0x3;
const APB_SARADC_SAR2_PATT_TAB1_REG: u32 = DR_REG_APB_SARADC_BASE + 0x28;
const APB_SARADC_SAR2_PATT_LEN_V: u32 = 0xF;
const APB_SARADC_SAR2_PATT_LEN_S: u32 = 19;
const APB_SARADC_SAR1_PATT_LEN_V: u32 = 0xF;
const APB_SARADC_SAR1_PATT_LEN_S: u32 = 15;
const APB_SARADC_SAR1_PATT_TAB1_REG: u32 = DR_REG_APB_SARADC_BASE + 0x18;
const SENS_SAR_MEAS1_MUX_REG: u32 = DR_REG_SENS_BASE + 0x10;
const SENS_SAR1_DIG_FORCE: u32 = 1 << 31;
const SENS_SAR_MEAS2_MUX_REG: u32 = DR_REG_SENS_BASE + 0x34;
const SENS_SAR2_RTC_FORCE: u32 = 1 << 31;
const APB_SARADC_APB_ADC_ARB_CTRL_REG: u32 = DR_REG_APB_SARADC_BASE + 0x38;
const APB_SARADC_ADC_ARB_GRANT_FORCE: u32 = 1 << 5;
const APB_SARADC_ADC_ARB_FIX_PRIORITY: u32 = 1 << 12;
const APB_SARADC_FILTER_CTRL0_REG: u32 = DR_REG_APB_SARADC_BASE + 0x3C;
const APB_SARADC_FILTER_CHANNEL0_V: u32 = 0x1F;
const APB_SARADC_FILTER_CHANNEL0_S: u32 = 19;
const APB_SARADC_FILTER_CHANNEL1_V: u32 = 0x1F;
const APB_SARADC_FILTER_CHANNEL1_S: u32 = 14;
const APB_SARADC_TIMER_SEL: u32 = 1 << 11;
const APB_SARADC_TIMER_EN: u32 = 1 << 24;
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
const SYSTEM_PERIP_RST_EN0_REG: u32 = DR_REG_SYSTEM_BASE + 0x20;
const SYSTEM_APB_SARADC_RST: u32 = 1 << 28;

use crate::regi2c_write_mask;

pub(crate) fn ensure_randomness() {
    set_peri_reg_mask(SYSTEM_WIFI_CLK_EN_REG, SYSTEM_WIFI_CLK_RNG_EN);

    // Enable 8M clock source for RNG (this is actually enough to produce strong
    // random results, but enabling the SAR ADC as well adds some insurance.)
    reg_set_bit(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_CLK8M_EN);

    // Enable SAR ADC to read a disconnected input for additional entropy
    set_peri_reg_mask(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_APB_SARADC_CLK_EN);
    clear_peri_reg_mask(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_APB_SARADC_CLK_EN);

    reg_set_field(
        APB_SARADC_APB_ADC_CLKM_CONF_REG,
        APB_SARADC_CLK_SEL_V,
        APB_SARADC_CLK_SEL_S,
        2,
    );

    set_peri_reg_mask(APB_SARADC_CTRL_REG, APB_SARADC_SAR_CLK_GATED);
    set_peri_reg_mask(APB_SARADC_APB_ADC_CLKM_CONF_REG, APB_SARADC_CLK_EN);

    reg_set_field(
        APB_SARADC_APB_ADC_CLKM_CONF_REG,
        APB_SARADC_CLKM_DIV_NUM_V,
        APB_SARADC_CLKM_DIV_NUM_S,
        3,
    );

    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SAR_CLK_DIV_V,
        APB_SARADC_SAR_CLK_DIV_S,
        3,
    );

    reg_set_field(
        APB_SARADC_CTRL2_REG,
        APB_SARADC_TIMER_TARGET_V,
        APB_SARADC_TIMER_TARGET_S,
        70,
    );

    clear_peri_reg_mask(APB_SARADC_CTRL_REG, APB_SARADC_START_FORCE);
    reg_set_field(
        SENS_SAR_POWER_XPD_SAR_REG,
        SENS_FORCE_XPD_SAR_V,
        SENS_FORCE_XPD_SAR_S,
        3,
    );
    clear_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_MEAS_NUM_LIMIT);

    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_WORK_MODE_V,
        APB_SARADC_WORK_MODE_S,
        1,
    );

    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SAR2_PATT_LEN_V,
        APB_SARADC_SAR2_PATT_LEN_S,
        0,
    );

    write_peri_reg(APB_SARADC_SAR2_PATT_TAB1_REG, 0xafffff);

    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SAR1_PATT_LEN_V,
        APB_SARADC_SAR1_PATT_LEN_S,
        0,
    );

    write_peri_reg(APB_SARADC_SAR1_PATT_TAB1_REG, 0xafffff);

    set_peri_reg_mask(SENS_SAR_MEAS1_MUX_REG, SENS_SAR1_DIG_FORCE);

    clear_peri_reg_mask(SENS_SAR_MEAS2_MUX_REG, SENS_SAR2_RTC_FORCE);

    clear_peri_reg_mask(
        APB_SARADC_APB_ADC_ARB_CTRL_REG,
        APB_SARADC_ADC_ARB_GRANT_FORCE,
    );
    clear_peri_reg_mask(
        APB_SARADC_APB_ADC_ARB_CTRL_REG,
        APB_SARADC_ADC_ARB_FIX_PRIORITY,
    );

    reg_set_field(
        APB_SARADC_FILTER_CTRL0_REG,
        APB_SARADC_FILTER_CHANNEL0_V,
        APB_SARADC_FILTER_CHANNEL0_S,
        0xD,
    );

    reg_set_field(
        APB_SARADC_FILTER_CTRL0_REG,
        APB_SARADC_FILTER_CHANNEL1_V,
        APB_SARADC_FILTER_CHANNEL1_S,
        0xD,
    );

    set_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_SEL);
    set_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_EN);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENCAL_REF_ADDR, 1);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_TSENS_ADDR, 1);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_RTC_ADDR, 1);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_DTEST_RTC_ADDR, 1);
}

pub fn revert_trng() {
    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENCAL_REF_ADDR, 0);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_TSENS_ADDR, 0);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_RTC_ADDR, 0);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_DTEST_RTC_ADDR, 0);

    reg_set_field(
        SENS_SAR_POWER_XPD_SAR_REG,
        SENS_FORCE_XPD_SAR_V,
        SENS_FORCE_XPD_SAR_S,
        0,
    );

    clear_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_EN);

    clear_peri_reg_mask(SYSTEM_PERIP_CLK_EN0_REG, APB_SARADC_CLK_EN);

    set_peri_reg_mask(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);
}

fn reg_set_bit(reg: u32, bit: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | bit);
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

const DR_REG_RTCCNTL_BASE: u32 = 0x60008000;
const RTC_CNTL_SENSOR_CTRL_REG: u32 = DR_REG_RTCCNTL_BASE + 0x108;
const RTC_CNTL_FORCE_XPD_SAR_V: u32 = 0x3;
const RTC_CNTL_FORCE_XPD_SAR_S: u32 = 30;
const RTC_CNTL_ANA_CONF_REG: u32 = DR_REG_RTCCNTL_BASE + 0x2c;
const RTC_CNTL_SAR_I2C_PU_M: u32 = 1 << 22;

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

const DR_REG_SYSTEM_BASE: u32 = 0x600c0000;
const SYSTEM_PERIP_CLK_EN0_REG: u32 = DR_REG_SYSTEM_BASE + 0x10;
const SYSTEM_PERIP_RST_EN0_REG: u32 = DR_REG_SYSTEM_BASE + 0x18;
const APB_SARADC_CLK_EN_M: u32 = 0x00000001 << 28;
const DR_REG_APB_SARADC_BASE: u32 = 0x60040000;
const APB_SARADC_APB_ADC_CLKM_CONF_REG: u32 = DR_REG_APB_SARADC_BASE + 0x54;
const APB_SARADC_REG_CLK_SEL_V: u32 = 0x00000003;
const APB_SARADC_REG_CLK_SEL_S: u32 = 21;
const APB_SARADC_CTRL_REG: u32 = DR_REG_APB_SARADC_BASE;
const APB_SARADC_SAR_PATT_P_CLEAR_M: u32 = 0x00000001 << 23;
const APB_SARADC_SAR_PATT_LEN_V: u32 = 0x00000007;
const APB_SARADC_SAR_PATT_LEN_S: u32 = 15;
const APB_SARADC_SAR_CLK_GATED_M: u32 = 0x00000001 << 6;
const APB_SARADC_XPD_SAR_FORCE_V: u32 = 0x00000003;
const APB_SARADC_XPD_SAR_FORCE_S: u32 = 27;
const APB_SARADC_SAR_CLK_DIV_V: u32 = 0x000000FF;
const APB_SARADC_SAR_CLK_DIV_S: u32 = 7;
const APB_SARADC_SAR_PATT_TAB1_REG: u32 = DR_REG_APB_SARADC_BASE + 0x18;
const APB_SARADC_SAR_PATT_TAB2_REG: u32 = DR_REG_APB_SARADC_BASE + 0x1c;
const APB_SARADC_SAR_PATT_TAB1_V: u32 = 0x00FFFFFF;
const APB_SARADC_SAR_PATT_TAB1_S: u32 = 0;
const APB_SARADC_SAR_PATT_TAB2_V: u32 = 0x00FFFFFF;
const APB_SARADC_SAR_PATT_TAB2_S: u32 = 0;
const APB_SARADC_CTRL2_REG: u32 = DR_REG_APB_SARADC_BASE + 0x4;
const APB_SARADC_TIMER_TARGET_V: u32 = 0x00000FFF;
const APB_SARADC_TIMER_TARGET_S: u32 = 12;
const APB_SARADC_REG_CLKM_DIV_NUM_V: u32 = 0x000000FF;
const APB_SARADC_REG_CLKM_DIV_NUM_S: u32 = 0;
const APB_SARADC_MEAS_NUM_LIMIT: u32 = 1 << 0;
const APB_SARADC_DMA_CONF_REG: u32 = DR_REG_APB_SARADC_BASE + 0x50;
const APB_SARADC_APB_ADC_TRANS_M: u32 = 0x00000001 << 31;
const APB_SARADC_TIMER_EN: u32 = 1 << 24;
const APB_SARADC_FSM_WAIT_REG: u32 = DR_REG_APB_SARADC_BASE + 0xc;
const APB_SARADC_RSTB_WAIT_V: u32 = 0x000000FF;
const APB_SARADC_RSTB_WAIT_S: u32 = 8;
const APB_SARADC_XPD_WAIT_V: u32 = 0x000000FF;
const APB_SARADC_XPD_WAIT_S: u32 = 0;
const APB_SARADC_STANDBY_WAIT_V: u32 = 0x000000FF;
const APB_SARADC_STANDBY_WAIT_S: u32 = 16;
const SYSTEM_APB_SARADC_RST_M: u32 = 0x00000001 << 28;
const SYSTEM_APB_SARADC_CLK_EN_M: u32 = 0x00000001 << 28;

use crate::regi2c_write_mask;

pub(crate) fn ensure_randomness() {
    // RNG module is always clock enabled
    reg_set_field(
        RTC_CNTL_SENSOR_CTRL_REG,
        RTC_CNTL_FORCE_XPD_SAR_V,
        RTC_CNTL_FORCE_XPD_SAR_S,
        0x3,
    );

    set_peri_reg_mask(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_SAR_I2C_PU_M);

    // Bridging sar2 internal reference voltage
    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC2_ENCAL_REF_ADDR, 1);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_DTEST_RTC_ADDR, 0);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_RTC_ADDR, 0);

    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC_ENT_TSENS_ADDR, 0);

    // Enable SAR ADC2 internal channel to read adc2 ref voltage for additional
    // entropy
    set_peri_reg_mask(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_APB_SARADC_CLK_EN_M);
    clear_peri_reg_mask(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST_M);
    reg_set_field(
        APB_SARADC_APB_ADC_CLKM_CONF_REG,
        APB_SARADC_REG_CLK_SEL_V,
        APB_SARADC_REG_CLK_SEL_S,
        0x2,
    );
    set_peri_reg_mask(APB_SARADC_APB_ADC_CLKM_CONF_REG, APB_SARADC_CLK_EN_M);
    set_peri_reg_mask(APB_SARADC_CTRL_REG, APB_SARADC_SAR_CLK_GATED_M);
    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_XPD_SAR_FORCE_V,
        APB_SARADC_XPD_SAR_FORCE_S,
        0x3,
    );
    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SAR_CLK_DIV_V,
        APB_SARADC_SAR_CLK_DIV_S,
        1,
    );

    reg_set_field(
        APB_SARADC_FSM_WAIT_REG,
        APB_SARADC_RSTB_WAIT_V,
        APB_SARADC_RSTB_WAIT_S,
        8,
    );

    reg_set_field(
        APB_SARADC_FSM_WAIT_REG,
        APB_SARADC_XPD_WAIT_V,
        APB_SARADC_XPD_WAIT_S,
        5,
    );

    reg_set_field(
        APB_SARADC_FSM_WAIT_REG,
        APB_SARADC_STANDBY_WAIT_V,
        APB_SARADC_STANDBY_WAIT_S,
        100,
    );

    set_peri_reg_mask(APB_SARADC_CTRL_REG, APB_SARADC_SAR_PATT_P_CLEAR_M);
    clear_peri_reg_mask(APB_SARADC_CTRL_REG, APB_SARADC_SAR_PATT_P_CLEAR_M);
    reg_set_field(
        APB_SARADC_CTRL_REG,
        APB_SARADC_SAR_PATT_LEN_V,
        APB_SARADC_SAR_PATT_LEN_S,
        0,
    );

    reg_set_field(
        APB_SARADC_SAR_PATT_TAB1_REG,
        APB_SARADC_SAR_PATT_TAB1_V,
        APB_SARADC_SAR_PATT_TAB1_S,
        0x9cffff,
    );
    reg_set_field(
        APB_SARADC_SAR_PATT_TAB2_REG,
        APB_SARADC_SAR_PATT_TAB2_V,
        APB_SARADC_SAR_PATT_TAB2_S,
        0x9cffff,
    );

    reg_set_field(
        APB_SARADC_CTRL2_REG,
        APB_SARADC_TIMER_TARGET_V,
        APB_SARADC_TIMER_TARGET_S,
        100,
    );
    reg_set_field(
        APB_SARADC_APB_ADC_CLKM_CONF_REG,
        APB_SARADC_REG_CLKM_DIV_NUM_V,
        APB_SARADC_REG_CLKM_DIV_NUM_S,
        15,
    );
    clear_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_MEAS_NUM_LIMIT);
    set_peri_reg_mask(APB_SARADC_DMA_CONF_REG, APB_SARADC_APB_ADC_TRANS_M);
    set_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_EN);
}

pub(crate) fn revert_trng() {
    regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC2_ENCAL_REF_ADDR, 0);
    clear_peri_reg_mask(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_EN);
    clear_peri_reg_mask(APB_SARADC_DMA_CONF_REG, APB_SARADC_APB_ADC_TRANS_M);
    reg_set_field(APB_SARADC_SAR_PATT_TAB1_REG, APB_SARADC_SAR_PATT_TAB1_V, APB_SARADC_SAR_PATT_TAB1_S, 0xffffff);
    reg_set_field(APB_SARADC_SAR_PATT_TAB2_REG, APB_SARADC_SAR_PATT_TAB2_V, APB_SARADC_SAR_PATT_TAB2_S, 0xffffff);
    clear_peri_reg_mask(APB_SARADC_APB_ADC_CLKM_CONF_REG, APB_SARADC_CLK_EN_M);
    reg_set_field(APB_SARADC_CTRL_REG, APB_SARADC_XPD_SAR_FORCE_V, APB_SARADC_XPD_SAR_FORCE_S, 0);
    reg_set_field(RTC_CNTL_SENSOR_CTRL_REG, RTC_CNTL_FORCE_XPD_SAR_V, RTC_CNTL_FORCE_XPD_SAR_S, 0);
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

const DR_REG_RTCCNTL_BASE: u32 = 0x3ff48000;
const RTC_CNTL_TEST_MUX_REG: u32 = DR_REG_RTCCNTL_BASE + 0xa8;
const RTC_CNTL_DTEST_RTC: u32 = 0x00000003;
const RTC_CNTL_DTEST_RTC_S: u32 = 30;
const RTC_CNTL_ENT_RTC: u32 = 1 << 29;
const SENS_SAR_START_FORCE_REG: u32 = 0x3ff48800 + 0x002c;
const SENS_SAR2_EN_TEST: u32 = 1 << 4;
const DPORT_PERIP_CLK_EN_REG: u32 = 0x3ff00000 + 0x0c0;
const DPORT_I2C_EXT0_CLK_EN: u32 = 1 << 7;

const DPORT_PERIP_RST_EN_REG: u32 = 0x3ff00000 + 0x0c4;
const DPORT_I2C_EXT0_RST: u32 = 1 << 7;

const SENS_ULP_CP_FORCE_START_TOP: u32 = 1 << 8;
const SENS_ULP_CP_START_TOP: u32 = 1 << 9;

const DR_REG_I2S_BASE: u32 = 0x3ff4f000;

const DR_REG_SYSCON_BASE: u32 = 0x3ff66000;
const SYSCON_SARADC_CTRL_REG: u32 = DR_REG_SYSCON_BASE + 0x10;
const SYSCON_SARADC_FSM_REG: u32 = DR_REG_SYSCON_BASE + 0x18;
const SYSCON_SARADC_SAR2_PATT_TAB1_REG: u32 = DR_REG_SYSCON_BASE + 0x2c;
const SYSCON_SARADC_SAR2_PATT_TAB2_REG: u32 = DR_REG_SYSCON_BASE + 0x30;
const SYSCON_SARADC_SAR2_PATT_TAB3_REG: u32 = DR_REG_SYSCON_BASE + 0x34;
const SYSCON_SARADC_SAR2_PATT_TAB4_REG: u32 = DR_REG_SYSCON_BASE + 0x38;
const SYSCON_SARADC_SAR2_MUX: u32 = 1 << 2;
const SYSCON_SARADC_SAR_CLK_DIV: u32 = 0x000000FF;
const SYSCON_SARADC_SAR_CLK_DIV_S: u32 = 7;
const SYSCON_SARADC_RSTB_WAIT: u32 = 0x000000FF;
const SYSCON_SARADC_RSTB_WAIT_S: u32 = 0;
const SYSCON_SARADC_START_WAIT: u32 = 1 << 2;
const SYSCON_SARADC_START_WAIT_S: u32 = 1 << 2;
const SYSCON_SARADC_WORK_MODE: u32 = 0x00000003;
const SYSCON_SARADC_WORK_MODE_S: u32 = 3;
const SYSCON_SARADC_SAR_SEL: u32 = 1 << 5;
const I2S_RX_BCK_DIV_NUM: u32 = 0x0000003F;
const I2S_RX_BCK_DIV_NUM_S: u32 = 6;
const SYSCON_SARADC_DATA_TO_I2S: u32 = 1 << 16;
const SYSCON_SARADC_DATA_SAR_SEL: u32 = 1 << 25;

const I2S_CAMERA_EN: u32 = 1 << 0;
const I2S_LCD_EN: u32 = 1 << 5;
const I2S_DATA_ENABLE: u32 = 1 << 4;
const I2S_DATA_ENABLE_TEST_EN: u32 = 1 << 3;
const I2S_RX_START: u32 = 1 << 5;
const I2S_RX_RESET: u32 = 1 << 1;

const DR_REG_SENS_BASE: u32 = 0x3ff48800;
const SENS_SAR_MEAS_WAIT2_REG: u32 = DR_REG_SENS_BASE + 0x000c;
const SENS_SAR_READ_CTRL_REG: u32 = DR_REG_SENS_BASE;
const SENS_SAR_READ_CTRL2_REG: u32 = DR_REG_SENS_BASE + 0x0090;
const SENS_FORCE_XPD_SAR: u32 = 0x00000003;
const SENS_FORCE_XPD_SAR_S: u32 = 18;
const SENS_SAR1_DIG_FORCE: u32 = 1 << 27;
const SENS_SAR2_DIG_FORCE: u32 = 1 << 28;

const I2S_CONF_REG0: u32 = DR_REG_I2S_BASE + 0x00a8;

pub fn ensure_randomness() {
    set_peri_reg_bits(
        RTC_CNTL_TEST_MUX_REG,
        RTC_CNTL_DTEST_RTC,
        2,
        RTC_CNTL_DTEST_RTC_S,
    );

    set_peri_reg_mask(RTC_CNTL_TEST_MUX_REG, RTC_CNTL_ENT_RTC);
    set_peri_reg_mask(SENS_SAR_START_FORCE_REG, SENS_SAR2_EN_TEST);

    // periph_module_enable(PERIPH_I2S0_MODULE);
    set_peri_reg_mask(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT0_CLK_EN);
    clear_peri_reg_mask(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT0_RST);

    clear_peri_reg_mask(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_FORCE_START_TOP);
    clear_peri_reg_mask(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_START_TOP);

    // Test pattern configuration byte 0xAD:
    //--[7:4] channel_sel: 10-->en_test
    //--[3:2] bit_width  : 3-->12bit
    //--[1:0] atten      : 1-->3dB attenuation
    write_peri_reg(SYSCON_SARADC_SAR2_PATT_TAB1_REG, 0xADADADAD);
    write_peri_reg(SYSCON_SARADC_SAR2_PATT_TAB2_REG, 0xADADADAD);
    write_peri_reg(SYSCON_SARADC_SAR2_PATT_TAB3_REG, 0xADADADAD);
    write_peri_reg(SYSCON_SARADC_SAR2_PATT_TAB4_REG, 0xADADADAD);

    set_peri_reg_bits(
        SENS_SAR_MEAS_WAIT2_REG,
        SENS_FORCE_XPD_SAR,
        3,
        SENS_FORCE_XPD_SAR_S,
    );

    set_peri_reg_mask(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DIG_FORCE);
    set_peri_reg_mask(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DIG_FORCE);

    set_peri_reg_mask(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_MUX);
    set_peri_reg_bits(
        SYSCON_SARADC_CTRL_REG,
        SYSCON_SARADC_SAR_CLK_DIV,
        4,
        SYSCON_SARADC_SAR_CLK_DIV_S,
    );
    set_peri_reg_bits(
        SYSCON_SARADC_FSM_REG,
        SYSCON_SARADC_RSTB_WAIT,
        8,
        SYSCON_SARADC_RSTB_WAIT_S,
    );
    set_peri_reg_bits(
        SYSCON_SARADC_FSM_REG,
        SYSCON_SARADC_START_WAIT,
        10,
        SYSCON_SARADC_START_WAIT_S,
    );
    set_peri_reg_bits(
        SYSCON_SARADC_CTRL_REG,
        SYSCON_SARADC_WORK_MODE,
        0,
        SYSCON_SARADC_WORK_MODE_S,
    );

    set_peri_reg_mask(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR_SEL);
    clear_peri_reg_mask(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_DATA_SAR_SEL);

    set_peri_reg_bits(
        DR_REG_I2S_BASE + 0x00b0,
        I2S_RX_BCK_DIV_NUM,
        20,
        I2S_RX_BCK_DIV_NUM_S,
    );

    set_peri_reg_mask(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_DATA_TO_I2S);

    clear_peri_reg_mask(DR_REG_I2S_BASE + 0x00a8, I2S_CAMERA_EN);
    set_peri_reg_mask(DR_REG_I2S_BASE + 0x00a8, I2S_LCD_EN);
    set_peri_reg_mask(DR_REG_I2S_BASE + 0x00a8, I2S_DATA_ENABLE);
    set_peri_reg_mask(DR_REG_I2S_BASE + 0x00a8, I2S_DATA_ENABLE_TEST_EN);
    set_peri_reg_mask(DR_REG_I2S_BASE + 0x00a8, I2S_RX_START);
}

pub fn revert_trng() {
    clear_peri_reg_mask(I2S_CONF_REG0, I2S_RX_START);
    set_peri_reg_mask(I2S_CONF_REG0, I2S_RX_RESET);
    clear_peri_reg_mask(I2S_CONF_REG0, I2S_RX_RESET);
    clear_peri_reg_mask(I2S_CONF_REG0, I2S_CAMERA_EN);
    clear_peri_reg_mask(I2S_CONF_REG0, I2S_LCD_EN);
    clear_peri_reg_mask(I2S_CONF_REG0, I2S_DATA_ENABLE_TEST_EN);
    clear_peri_reg_mask(I2S_CONF_REG0, I2S_DATA_ENABLE);

    clear_peri_reg_mask(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DIG_FORCE);
    clear_peri_reg_mask(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DIG_FORCE);

    clear_peri_reg_mask(SENS_SAR_START_FORCE_REG, SENS_SAR2_EN_TEST);
    clear_peri_reg_mask(
        SYSCON_SARADC_CTRL_REG,
        SYSCON_SARADC_SAR2_MUX | SYSCON_SARADC_SAR_SEL | SYSCON_SARADC_DATA_TO_I2S,
    );

    set_peri_reg_bits(
        SENS_SAR_MEAS_WAIT2_REG,
        SENS_FORCE_XPD_SAR,
        0,
        SENS_FORCE_XPD_SAR_S,
    );

    set_peri_reg_bits(
        SYSCON_SARADC_FSM_REG,
        SYSCON_SARADC_START_WAIT,
        8,
        SYSCON_SARADC_START_WAIT_S,
    );
}

fn set_peri_reg_bits(reg: u32, bitmap: u32, value: u32, shift: u32) {
    unsafe {
        (reg as *mut u32).write_volatile(
            ((reg as *mut u32).read_volatile() & !(bitmap << shift)) | ((value & bitmap) << shift),
        );
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

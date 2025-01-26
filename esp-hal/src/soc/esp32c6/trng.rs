use crate::peripherals::{APB_SARADC, PCR, PMU};

const I2C_SAR_ADC: u8 = 0x69;
const I2C_SAR_ADC_HOSTID: u8 = 0;

const SAR2_CHANNEL: u32 = 9;
const SAR2_ATTEN: u32 = 1;
const SAR1_ATTEN: u32 = 1;
const PATTERN_BIT_WIDTH: u32 = 6;

const REGI2C_BBPLL: u8 = 0x66;
const REGI2C_BIAS: u8 = 0x6a;
const REGI2C_DIG_REG: u8 = 0x6d;
const REGI2C_ULP_CAL: u8 = 0x61;
const REGI2C_SAR_I2C: u8 = 0x69;

const I2C_MST_ANA_CONF1_M: u32 = 0x00FFFFFF;
const REGI2C_BBPLL_RD_MASK: u32 = !(1 << 7) & I2C_MST_ANA_CONF1_M;
const REGI2C_BIAS_RD_MASK: u32 = !(1 << 6) & I2C_MST_ANA_CONF1_M;
const REGI2C_DIG_REG_RD_MASK: u32 = !(1 << 10) & I2C_MST_ANA_CONF1_M;
const REGI2C_ULP_CAL_RD_MASK: u32 = !(1 << 8) & I2C_MST_ANA_CONF1_M;
const REGI2C_SAR_I2C_RD_MASK: u32 = !(1 << 9) & I2C_MST_ANA_CONF1_M;

const REGI2C_RTC_SLAVE_ID_V: u8 = 0xFF;
const REGI2C_RTC_SLAVE_ID_S: u8 = 0;
const REGI2C_RTC_ADDR_V: u8 = 0xFF;
const REGI2C_RTC_ADDR_S: u8 = 8;
const REGI2C_RTC_WR_CNTL_V: u8 = 0x1;
const REGI2C_RTC_WR_CNTL_S: u8 = 24;
const REGI2C_RTC_DATA_V: u8 = 0xFF;
const REGI2C_RTC_DATA_S: u8 = 16;

const ADC_SAR2_INITIAL_CODE_HIGH_ADDR: u8 = 0x4;
const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_MSB: u8 = 0x3;
const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_LSB: u8 = 0x0;

const ADC_SAR2_INITIAL_CODE_LOW_ADDR: u8 = 0x3;
const ADC_SAR2_INITIAL_CODE_LOW_ADDR_MSB: u8 = 0x7;
const ADC_SAR2_INITIAL_CODE_LOW_ADDR_LSB: u8 = 0x0;

const ADC_SAR1_INITIAL_CODE_HIGH_ADDR: u8 = 0x1;
const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB: u8 = 0x3;
const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB: u8 = 0x0;

const ADC_SAR1_INITIAL_CODE_LOW_ADDR: u8 = 0x0;
const ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB: u8 = 0x7;
const ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB: u8 = 0x0;

const ADC_SARADC_DTEST_RTC_ADDR: u8 = 0x7;
const ADC_SARADC_DTEST_RTC_ADDR_MSB: u8 = 1;
const ADC_SARADC_DTEST_RTC_ADDR_LSB: u8 = 0;

const ADC_SARADC_ENT_RTC_ADDR: u8 = 0x7;
const ADC_SARADC_ENT_RTC_ADDR_MSB: u8 = 3;
const ADC_SARADC_ENT_RTC_ADDR_LSB: u8 = 3;

const ADC_SARADC1_ENCAL_REF_ADDR: u8 = 0x7;
const ADC_SARADC1_ENCAL_REF_ADDR_MSB: u8 = 4;
const ADC_SARADC1_ENCAL_REF_ADDR_LSB: u8 = 4;

const ADC_SARADC2_ENCAL_REF_ADDR: u8 = 0x7;
const ADC_SARADC2_ENCAL_REF_ADDR_MSB: u8 = 6;
const ADC_SARADC2_ENCAL_REF_ADDR_LSB: u8 = 6;

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
        pmu.rf_pwc().modify(|_, w| w.perif_i2c_rstb().set_bit());

        pmu.rf_pwc().modify(|_, w| w.xpd_perif_i2c().set_bit());

        // Config ADC circuit (Analog part) with I2C(HOST ID 0x69) and chose internal
        // voltage as sampling source
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_DTEST_RTC_ADDR,
            ADC_SARADC_DTEST_RTC_ADDR_MSB,
            ADC_SARADC_DTEST_RTC_ADDR_LSB,
            2,
        );
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENT_RTC_ADDR,
            ADC_SARADC_ENT_RTC_ADDR_MSB,
            ADC_SARADC_ENT_RTC_ADDR_LSB,
            1,
        );
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC1_ENCAL_REF_ADDR,
            ADC_SARADC1_ENCAL_REF_ADDR_MSB,
            ADC_SARADC1_ENCAL_REF_ADDR_LSB,
            1,
        );
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC2_ENCAL_REF_ADDR,
            ADC_SARADC2_ENCAL_REF_ADDR_MSB,
            ADC_SARADC2_ENCAL_REF_ADDR_LSB,
            1,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR_MSB,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR_LSB,
            0x08,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR_MSB,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR_LSB,
            0x66,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB,
            0x08,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB,
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
        apb_saradc.ctrl().modify(|_, w| w.sar_patt_len().bits(1));

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

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR_MSB,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR_LSB,
            0x60,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR_MSB,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB,
            0x60,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_DTEST_RTC_ADDR,
            ADC_SARADC_DTEST_RTC_ADDR_MSB,
            ADC_SARADC_DTEST_RTC_ADDR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC_ENT_RTC_ADDR,
            ADC_SARADC_ENT_RTC_ADDR_MSB,
            ADC_SARADC_ENT_RTC_ADDR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC1_ENCAL_REF_ADDR,
            ADC_SARADC1_ENCAL_REF_ADDR_MSB,
            ADC_SARADC1_ENCAL_REF_ADDR_LSB,
            0,
        );

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SARADC2_ENCAL_REF_ADDR,
            ADC_SARADC2_ENCAL_REF_ADDR_MSB,
            ADC_SARADC2_ENCAL_REF_ADDR_LSB,
            0,
        );

        PCR::regs()
            .saradc_clkm_conf()
            .modify(|_, w| w.bits(0x00404000));

        PCR::regs().saradc_conf().modify(|_, w| w.bits(0x5));
    }
}

fn regi2c_enable_block(block: u8) {
    let modem_lpcon = crate::peripherals::MODEM_LPCON::regs();
    let lp_i2c_ana = crate::peripherals::LP_I2C_ANA_MST::regs();

    unsafe {
        modem_lpcon
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().set_bit());

        modem_lpcon
            .i2c_mst_clk_conf()
            .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());

        lp_i2c_ana
            .date()
            .modify(|_, w| w.lp_i2c_ana_mast_i2c_mat_clk_en().set_bit());

        match block {
            REGI2C_BBPLL => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() | REGI2C_BBPLL_RD_MASK)
                });
            }
            REGI2C_BIAS => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() | REGI2C_BIAS_RD_MASK)
                });
            }
            REGI2C_DIG_REG => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() | REGI2C_DIG_REG_RD_MASK)
                });
            }
            REGI2C_ULP_CAL => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() | REGI2C_ULP_CAL_RD_MASK)
                });
            }
            REGI2C_SAR_I2C => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() | REGI2C_SAR_I2C_RD_MASK)
                });
            }
            _ => (),
        }
    }
}

fn regi2c_disable_block(block: u8) {
    let lp_i2c_ana = crate::peripherals::LP_I2C_ANA_MST::regs();

    unsafe {
        match block {
            REGI2C_BBPLL => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() & !REGI2C_BBPLL_RD_MASK)
                });
            }
            REGI2C_BIAS => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() & !REGI2C_BIAS_RD_MASK)
                });
            }
            REGI2C_DIG_REG => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() & !REGI2C_DIG_REG_RD_MASK)
                });
            }
            REGI2C_ULP_CAL => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() & !REGI2C_ULP_CAL_RD_MASK)
                });
            }
            REGI2C_SAR_I2C => {
                lp_i2c_ana.ana_conf1().modify(|r, w| {
                    w.lp_i2c_ana_mast_ana_conf1()
                        .bits(r.lp_i2c_ana_mast_ana_conf1().bits() & !REGI2C_SAR_I2C_RD_MASK)
                });
            }
            _ => (),
        }
    }
}

pub(crate) fn regi2c_write_mask(block: u8, _host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    assert!(msb - lsb < 8);
    let lp_i2c_ana = crate::peripherals::LP_I2C_ANA_MST::regs();

    unsafe {
        regi2c_enable_block(block);

        // Read the i2c bus register
        let mut temp: u32 = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32)
            << REGI2C_RTC_SLAVE_ID_S as u32)
            | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32);

        lp_i2c_ana
            .i2c0_ctrl()
            .modify(|_, w| w.lp_i2c_ana_mast_i2c0_ctrl().bits(temp));

        while lp_i2c_ana
            .i2c0_ctrl()
            .read()
            .lp_i2c_ana_mast_i2c0_busy()
            .bit()
        {}

        temp = lp_i2c_ana
            .i2c0_data()
            .read()
            .lp_i2c_ana_mast_i2c0_rdata()
            .bits() as u32;

        // Write the i2c bus register
        temp &= (!(0xFFFFFFFF << lsb)) | (0xFFFFFFFF << (msb + 1));
        temp |= (data as u32 & (!(0xFFFFFFFF << (msb as u32 - lsb as u32 + 1)))) << (lsb as u32);
        temp = ((block as u32 & REGI2C_RTC_SLAVE_ID_V as u32) << REGI2C_RTC_SLAVE_ID_S as u32)
            | ((reg_add as u32 & REGI2C_RTC_ADDR_V as u32) << REGI2C_RTC_ADDR_S as u32)
            | ((0x1 & REGI2C_RTC_WR_CNTL_V as u32) << REGI2C_RTC_WR_CNTL_S as u32)
            | ((temp & REGI2C_RTC_DATA_V as u32) << REGI2C_RTC_DATA_S as u32);

        lp_i2c_ana
            .i2c0_ctrl()
            .modify(|_, w| w.lp_i2c_ana_mast_i2c0_ctrl().bits(temp));

        while lp_i2c_ana
            .i2c0_ctrl()
            .read()
            .lp_i2c_ana_mast_i2c0_busy()
            .bit()
        {}

        regi2c_disable_block(block);
    }
}
